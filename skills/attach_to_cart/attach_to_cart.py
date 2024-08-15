import time

from raya.skills import RayaFSMSkill
from raya.tools.filesystem import create_dat_folder
from raya.controllers.motion_controller import MotionController
from raya.controllers.sound_controller import SoundController
from raya.controllers.sensors_controller import SensorsController
from raya.controllers.robot_skills_controller import RobotSkillsController
from raya.exceptions import RayaUnknownServerError
from raya.skills.skill import RayaSkillHandler

from raya.exceptions import RayaSkillAborted

from .constants import *

from .attach.attach import AttachToCart
from .attach.constants import *

class SkillAttachToCart(RayaFSMSkill):
    
    DEFAULT_SETUP_ARGS = {
        'timeout' : FULL_APP_TIMEOUT,
        '180_rotating': DEFUALT_ROTATING_180,
        'actual_desired_position': GRIPPER_ACTUAL_DESIRED_POSITION,
        'reverse_beeping_alert': REVERSE_BEEPING_ALERT,
        'close_pressure': GRIPPER_CLOSE_PRESSURE_CONST,
    }
    
    REQUIRED_SETUP_ARGS = {}
    
    DEFAULT_EXECUTE_ARGS = {
        'family': FAMILY,
        'target_distance': TARGET_DISTANCE,
        'reverse': REVERSE,
        'wait_time_for_detection': WAIT_TIME_FOR_DETECTION,
    }
    
    REQUIRED_EXECUTE_ARGS = {
        'tag_size',
        'target_tags'
    }


###############################################################################
###########################    FSM states     #################################
###############################################################################

    STATES = [
        'SETUP',
        'SEARCHING_TAGS',
        'APPROACH',
        'ATTACH',
        'END',
    ]

    INITIAL_STATE = 'SETUP'

    END_STATES = [
        'END'
    ]
    
    STATES_TIMEOUTS = {}


###############################################################################
###########################   skill methods   #################################
###############################################################################


    async def setup(self):
        self.sensors:SensorsController = await self.enable_controller('sensors')
        self.motion:MotionController = await self.enable_controller('motion')
        self.sound:SoundController = await self.enable_controller('sound')
        self.cameras: CamerasController = \
                await self.enable_controller('cameras')
        self.cv:CVController = await self.enable_controller('cv')
        self.robot_skills:RobotSkillsController = \
            await self.enable_controller('robot_skills')
        
        ## create folder for audio
        create_dat_folder(AUDIO_PATH)
        
        i_time = time.time()
        self.log.info('Registering AttachToCart Skill...')
        self.attach:RayaSkillHandler = self.register_skill(AttachToCart)
        await self.attach.execute_setup(self.setup_args)
        self.log.info(f'AttachToCart setup took:{time.time() - i_time} seconds')

    
    async def finish(self):
        pass


###############################################################################
###########################      helpers      #################################
###############################################################################


    async def approach(self):
        try:
            if SKILL_NAME == 'approach_to_tag': law = 0.15
            else: law = 0.07
            result = await self.robot_skills.execute_skill(
                skill=SKILL_NAME,
                callback_feedback_async=self.cb_feedback_skill,
                wait=True,
                family=self.execute_args['family'],
                tag_size=self.execute_args['tag_size'],
                sources=self.sources,
                target_tags=[self.selected_tag],
                target_distance=self.execute_args['target_distance'],
                wait_target_time=30.0,
                reverse=self.execute_args['reverse'],
                max_x_error= 0.02, 
                max_y_error= 0.02,
                low_angular_velocity=law,
                high_linear_velocity=0.2,
                min_approach_distance=0.5
            )
        except RayaUnknownServerError as e:
            self.log.warn(f'///////////////')
            self.log.warn(f'Skill Failed:' )
            self.log.warn(f'  error_code:    {e.error_code}' )
            self.log.warn(f'  error_message: {e.error_msg}' )
            self.abort(error_code=e.error_code, error_msg=e.error_msg)
        else:
            self.log.warn(f'///////////////')
            self.log.warn(f'Skill Finished:' )
            self.log.warn(f'///////////////')
            self.log.info(f'X Error: {result[0]}')
            self.log.info(f'Y Error: {result[1]}')
            self.log.info(f'Angle Error: {result[2]}') 


    async def cb_feedback_skill(self, 
            feedback_code,
            feedback_msg,
            x_error,
            y_error,
            angle_error
        ):
        self.log.debug(f'----------------------')
        self.log.debug(f'Feedback Code: {feedback_code}' )
        self.log.debug(f'Feedback State: {feedback_msg}' )
        self.log.debug(f'Current Error: ')
        self.log.debug(f'X: {x_error}')
        self.log.debug(f'Y: {y_error}')
        self.log.debug(f'Angle: {angle_error}')
        self.log.debug(f'----------------------')


    async def select_best_tag(self):
        self.app.log.info('Selecting a tag...')
        start_time = time.time()
        while True:
            if time.time() - start_time > self.execute_args['wait_time_for_detection']:
                break
            
            if len(self.view_tags.keys()) > 0:
                self.log.debug('Tags detected')
                break
            await self.sleep(0.1)
        
        tags = copy.copy(self.view_tags)
        if len(tags.keys()) == 0:
            self.app.log.error(
                'No tags detected were detected after '
                f'{self.execute_args["wait_time_for_detection"]} seconds, '
                'aborting...'
            )
            self.abort(*ERROR_NO_TAGS_DETECTED)

        centerest_tag = None
        for tag in tags:
            self.log.warn(f'Tag detected: {tag}')
            if centerest_tag is None:
                centerest_tag = tag
            else:
                if abs(tags[tag].y) < abs(tags[centerest_tag].y):
                    centerest_tag = tag
        
        self.log.info('Disabling model...')
        await self.cv.disable_model(model_obj=self.detector)
        self.log.info('Disabling cameras...')
        for camera in self.sources:
            await self.cameras.disable_camera(camera_name=camera)
        
        if centerest_tag is None:
            self.abort(*ERROR_NO_TAGS_DETECTED)
        self.app.log.info(f'Selected tag: {centerest_tag}')
        return centerest_tag


    def callback_all_predictions(self, detections, image):
        if detections:
            for tag in detections:
                id = str(tag['tag_id'])
                if id not in self.execute_args['target_tags']:
                    continue
                self.view_tags[id] = tag["pose_base_link"].pose.position


    async def select_best_tag(self):
        self.app.log.info('Selecting a tag...')
        start_time = time.time()
        while True:
            if time.time() - start_time > self.execute_args['wait_time_for_detection']:
                break
            
            if len(self.view_tags.keys()) > 0:
                self.log.debug('Tags detected')
                break
            await self.sleep(0.1)
        
        tags = copy.copy(self.view_tags)
        if len(tags.keys()) == 0:
            self.app.log.error(
                'No tags detected were detected after '
                f'{self.execute_args["wait_time_for_detection"]} seconds, '
                'aborting...'
            )
            self.abort(*ERROR_NO_TAGS_DETECTED)

        centerest_tag = None
        for tag in tags:
            self.log.warn(f'Tag detected: {tag}')
            if centerest_tag is None:
                centerest_tag = tag
            else:
                if abs(tags[tag].y) < abs(tags[centerest_tag].y):
                    centerest_tag = tag
        
        self.log.info('Disabling model...')
        await self.cv.disable_model(model_obj=self.detector)
        self.log.info('Disabling cameras...')
        for camera in self.sources:
            await self.cameras.disable_camera(camera_name=camera)
        
        if centerest_tag is None:
            self.abort(*ERROR_NO_TAGS_DETECTED)
        self.app.log.info(f'Selected tag: {centerest_tag}')
        return centerest_tag


    def callback_all_predictions(self, detections, image):
        if detections:
            for tag in detections:
                id = str(tag['tag_id'])
                if id not in self.execute_args['target_tags']:
                    continue
                self.view_tags[id] = tag["pose_base_link"].pose.position


###############################################################################
#########################      ACTIONS       ##################################
###############################################################################


    async def enter_SETUP(self):
        self.log.debug('Entered SETUP state')
        if self.execute_args['reverse']:
            self.sources = ['back']
        else:
            self.sources = ['nav_bottom', 'nav_top']

        self.app.log.debug(f'Used sources: {self.sources}')
        self.app.log.debug(f'Used tags: {self.execute_args["target_tags"]}')
        
        self.app.log.info('Enabling cameras...')
        for camera in self.sources:
            await self.cameras.enable_camera(camera_name=camera)
        
        self.app.log.info('Enabling cv...')
        MODEL_PARAMS['tag_size'] = self.execute_args['tag_size']
        
        # Enable detector
        start_time = time.time()
        self.log.info('Enabling model...')
        self.detector: TagsDetectorHandler = await self.cv.enable_model(
                model='detector',type='tag',
                name='apriltags', 
                source=self.sources[0],
                model_params = MODEL_PARAMS
            )
        self.log.info(f'Model enabled {str(time.time()-start_time)}')
        
        # Create listener
        self.view_tags = dict()
        self.detector.set_img_predictions_callback(
                callback=self.callback_all_predictions,
                as_dict=True,
                cameras_controller=self.cameras
            )


    async def enter_SEARCHING_TAGS(self):
        self.app.log.info('Entered SEARCHING_TAGS state')
        self.selected_tag = await self.select_best_tag()
        

    async def enter_APPROACH(self):
        self.log.debug('Entered APPROACH state')
        await self.approach()
        self.log.debug('Executed APPROACH state')


    async def enter_ATTACH(self):
        self.log.debug('Entered ATTACH state')
        try:
            result = await self.attach.execute_main()
        except RayaSkillAborted as e:
            self.log.error(f'///////////////')
            self.log.error(f'Attach Failed:' )
            self.log.error(f'  error_code:    {e.error_code}' )
            self.log.error(f'  error_message: {e.error_msg}' )
            self.abort(error_code=e.error_code, error_msg=e.error_msg)
        else:
            self.log.error(f'///////////////')
            self.log.error(f'Attach Finished:' )
            self.log.error(f'///////////////')
            self.log.error(f'X Error: {result[0]}')
            self.log.error(f'Y Error: {result[1]}')
        self.log.debug('Executed ATTACH state')


    async def enter_END(self):
        self.log.info('Entered END state')
        self.log.info('RayaSkill.finish')


###############################################################################
#########################    TRASITIONS      ##################################
###############################################################################

    async def transition_from_SETUP(self):
        self.set_state('SEARCHING_TAGS')


    async def transition_from_SEARCHING_TAGS(self):
        self.set_state('APPROACH')


    async def transition_from_APPROACH(self):
        self.set_state('ATTACH')


    async def transition_from_ATTACH(self):
        self.set_state('END')

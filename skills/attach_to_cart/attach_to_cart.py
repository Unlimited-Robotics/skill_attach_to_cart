import time

from raya.skills import RayaFSMSkill
from raya.tools.filesystem import create_dat_folder
from raya.controllers.arms_controller import ArmsController
from raya.controllers.motion_controller import MotionController
from raya.controllers.sound_controller import SoundController
from raya.controllers.sensors_controller import SensorsController
from raya.controllers.robot_skills_controller import RobotSkillsController
from raya.exceptions import RayaUnknownServerError
from raya.skills.skill import RayaSkillHandler

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
        self.arms:ArmsController = await self.enable_controller('arms')
        self.sensors:SensorsController = await self.enable_controller('sensors')
        self.motion:MotionController = await self.enable_controller('motion')
        self.sound:SoundController = await self.enable_controller('sound')
        self.robot_skills:RobotSkillsController = \
            await self.enable_controller('robot_skills')
        
        ## create folder for audio
        create_dat_folder(AUDIO_PATH)
        
        i_time = time.time()
        self.app.log.info('Registering AttachToCart Skill...')
        self.attach:RayaSkillHandler = self.register_skill(AttachToCart)
        await self.attach.execute_setup(self.setup_args)
        self.app.log.info(f'AttachToCart setup took:{time.time() - i_time} seconds')      

    
    async def finish(self):
        pass


###############################################################################
###########################      helpers      #################################
###############################################################################


    async def approach(self):
        if self.setup_args['reverse']:
            self.sources = ['back']
        else:
            self.sources = ['nav_bottom', 'nav_top']

        self.app.log.debug(f'Used sources: {self.sources}')

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
                target_tags=self.execute_args['target_tags'],
                target_distance=self.execute_args['target_distance'],
                reverse=self.execute_args['reverse'],
                max_x_error= 0.03, 
                max_y_error= 0.03,
                low_angular_velocity=law
            )
        except RayaUnknownServerError as e:
            self.app.log.warn(f'///////////////')
            self.app.log.warn(f'Skill Failed:' )
            self.app.log.warn(f'  error_code:    {e.error_code}' )
            self.app.log.warn(f'  error_message: {e.error_msg}' )
        else:
            self.app.log.warn(f'///////////////')
            self.app.log.warn(f'Skill Finished:' )
            self.app.log.warn(f'///////////////')
            self.app.log.info(f'X Error: {result[0]}')
            self.app.log.info(f'Y Error: {result[1]}')
            self.app.log.info(f'Angle Error: {result[2]}') 


    async def cb_feedback_skill(self, 
            feedback_code,
            feedback_msg,
            x_error,
            y_error,
            angle_error
        ):
        self.app.log.warn(f'')
        self.app.log.warn(f'Feedback State: {feedback_msg}' )
        self.app.log.warn(f'')
        self.app.log.info(f'')
        self.app.log.info(f'Current Error: ')
        self.app.log.info(f'')
        self.app.log.info(f'X: {x_error}')
        self.app.log.info(f'Y: {y_error}')
        self.app.log.info(f'Angle: {angle_error}')


###############################################################################
#########################      ACTIONS       ##################################
###############################################################################


    async def enter_SETUP(self):
        self.app.log.info('Entered SETUP state')
        

    async def enter_APPROACH(self):
        self.app.log.info('Entered APPROACH state')
        await self.approach() 
        # raise Exception('Finished approach')


    async def enter_ATTACH(self):
        self.app.log.info('Entered ATTACH state')
        if not self.execute_args['reverse']:
            self.motion.rotate(180.0, wait=True)
        await self.attach.execute_main()


    async def enter_END(self):
        self.app.log.info('Entered END state')


###############################################################################
#########################    TRASITIONS      ##################################
###############################################################################

    async def transition_SETUP(self):
        self.set_state('APPROACH')


    async def transition_APPROACH(self):
        self.set_state('ATTACH')


    async def transition_ATTACH(self):
        self.set_state('END')

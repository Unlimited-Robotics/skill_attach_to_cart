from raya.skills import RayaSkill, RayaSkillHandler
from raya.controllers import MotionController
from raya.application_base import RayaApplicationBase
import asyncio
from .constants import *
import math
import time
import eyed3
from raya.tools.filesystem import resolve_path, create_dat_folder



class SkillAttachToCart(RayaSkill):

    DEFAULT_SETUP_ARGS = {

        'timeout' : FULL_APP_TIMEOUT,
        '180_rotating': DEFUALT_ROTATING_180,
        'actual_desired_position': GRIPPER_ACTUAL_DESIRED_POSITION,
        'reverse_beeping_alert': REVERSE_BEEPING_ALERT
            }
    REQUIRED_SETUP_ARGS = {
         
    }
    
###############################################################################
########################### state classifiers #################################
###############################################################################

    async def state_classifier(self):
        ### change to parameters
        ## rotating state
        ## If self.dl is less then rotating disance or right
        # and also the sum is less then average
        # and self.angle is above rotating..

        self.log.debug(f'current state: {self.state}')

        if (self.state == 'attach_verification'):
            return True

        elif (self.state == 'finish'):
            return True
        
        elif ((self.dl<ROTATING_DISTANCE or self.dr<ROTATING_DISTANCE) and\
             (self.dl+self.dr)/2 < ROTATING_DISTANCE_AV and\
                  abs(self.angle) > ROTATING_ANGLE_MIN):

            self.state = 'rotating'
            return True
        
        ## If the sensor distance is low then min every thing ok you can close
        ## If the distance is lower then max size and also the orientation angle is low - close ok
        elif ((self.dl < ATACHING_DISTANCE_MIN and\
              self.dr < ATACHING_DISTANCE_MIN) or\
                  (self.dl<ATACHING_DISTANCE_MAX and\
                    self.dr<ATACHING_DISTANCE_MAX and\
                          abs(self.angle)<ATACHING_ANGLE_MAX)):
            self.state = 'attaching'
            return True
        
        else:
            self.state = 'moving'
            return True
        

    async def gripper_state_classifier(self):

        if (self.gripper_state['pressure_reached'] == True and \
            self.gripper_state['position_reached'] == False):

            # If the pressure was reached but the position wasnt reached, that
            # means the adapter touched something. Check if the adapter is close
            # to the actual desired position and mark the cart as attached
            if self.gripper_state['close_to_actual_position'] == True:
                self.gripper_state['cart_attached'] = True

            # If its not, try to attach again
            else:
                await self.send_feedback('Actual desired position not reached. Attaching again...')
                self.state = 'attaching'

        else:
            self.gripper_state['cart_attached'] = False
            self.state = 'finish'
###############################################################################
###############################################################################
###############################################################################

###############################################################################
############################# feedback & callback #############################
###############################################################################
                
    async def gripper_feedback_cb(self, gripper_result):

        ## INPUT: gripper feednack result from raya
        ## function updates local list of parameters which define the gripper state

        self.gripper_state['final_position'] =  gripper_result['final_position']
        self.gripper_state['final_pressure'] = gripper_result['final_pressure']
        self.gripper_state['position_reached'] = gripper_result['position_reached']
        self.gripper_state['pressure_reached'] = gripper_result['pressure_reached']
        self.gripper_state['success'] = gripper_result['success']
        self.gripper_state['timeout_reached'] = gripper_result['timeout_reached']
        if abs(gripper_result['final_position'] - self.setup_args['actual_desired_position']) < POSITION_ERROR_MARGIN: 
            self.gripper_state['close_to_actual_position'] = True
        else:
            self.log.info(f'Attemps,{self.gripper_state["attempts"]}, final_position {gripper_result["final_position"]}')
        
    def cb_feedback_sound(self, error, error_msg, distance):
        pass
    
    def cb_finish_sound(self, status, status_msg):
            pass
    
###############################################################################
###############################################################################
###############################################################################

###############################################################################
################### Un-expected senerio detections and actions ################
###############################################################################

    async def obstacle_detection(self):
        ### function detects if an object entered between gary and the cart
        ### and if the object disapeared. function raises flag accordinly
        if abs(self.last_middle_srf / self.middle_srf) > OBSTACLE_DETECTED_RATIO:
            self.log.debug(f'obstacle_detected: {self.obstacle_detected}, ratio: {self.last_middle_srf / self.middle_srf}')
            self.obstacle_detected = True
        elif self.obstacle_detected:
            if abs(self.last_middle_srf / self.middle_srf) < NON_OBSTACLE_DETECTED_RATIO:
                self.log.debug(f'obstacle_detected: {self.obstacle_detected}, ratio: {self.last_middle_srf / self.middle_srf}')
                self.obstacle_detected = False

        self.last_middle_srf = self.middle_srf

    async def avoid_obstacle(self):
        #### function activated if object or humen detected suddenly between the robot and cart
        ### the function stops gary from moving, alert with beeping sound and apply sleep method for OBSTACLE_SLEEPING_TIME seconds.
        ### after MAX_OBSTACLE_INDEX times of identification, the function abort the skill

        is_moving = self.motion.is_moving()
        if (is_moving):
            await self.motion.cancel_motion()
            if REVERSE_BEEPING_ALERT:
                if not self.sound.is_playing():
                        await self.play_predefined_sound('beep', leds = False, wait = False)
      
        self.obstacle_index = self.obstacle_index + 1
        self.log.error(f'stop moving, obstacle detected {self.middle_srf} cm from gary, index: {self.obstacle_index}')
        if self.obstacle_index > MAX_OBSTACLE_INDEX:
            self.log.error(f'error, max obstacle index reached: {self.obstacle_index}')
            if self.sound.is_playing():
                self.sound.cancel_sound()
            self.abort(*ERROR_OBSTACLE_IDENTIFIED)
        await self.sleep(OBSTACLE_SLEEPING_TIME)


    async def vibrate(self):
            ### function move gary to help the gripper to close all the way
            try:
                await self.motion.set_velocity(
                        x_velocity = VERIFICATION_VELOCITY,
                        y_velocity = 0.0,
                        angular_velocity=0.0,
                        duration=0.5,
                        enable_obstacles=False,
                        wait=True, 
                        )
                await self.motion.set_velocity(
                        x_velocity = -VERIFICATION_VELOCITY,
                        y_velocity = 0.0,
                        angular_velocity=0.0,
                        duration=0.3,
                        enable_obstacles=False,
                        wait=True, 
                        )
            except Exception as error:
                self.log.error(f'linear movement failed, error: {error}')
                self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)

    async def pushing_cart_identifier(self):
        self.last_average_distance = self.average_distance
        await self.read_srf_values()
        
        if self.average_distance > self.last_average_distance:
            self.pushing_index += 1
            self.log.warn(f'cart seems to be pushed by gary, index: {self.pushing_index} '\
                          f'av_dis: {self.average_distance}, last av_dis: {self.last_average_distance}')
        if self.pushing_index > MAX_PUSHING_INDEX:
            self.log.error(f'cart pushed by gary {self.pushing_index} times')
            self.abort(*ERROR_CART_NOT_GETTING_CLOSER)

    async def _cart_max_distance_verification (self):
            if self.dl > CART_MAX_DISTANCE and self.dr > CART_MAX_DISTANCE:
                self.log.error(f'cart is too far, distance: left: {self.dl} cm, right: {self.dr}')
                self.abort(*ERROR_CART_NOT_ACCESSABLE)


    async def major_angle_correction (self):
        index = 0
        while abs(self.angle) > MIN_STARTING_ANGLE and index < MAX_ANGLE_CORRECTION_ATTEMPTS:
            self.log.info(f'major angle correction attempt: {index}')
            await self._timer_update()
            await self._timeout_verification()
            await self.adjust_angle()
            await self.read_srf_values()
            await self.calculate_distance_parameters()
            await self._cart_max_distance_verification()
            index+=1
    

    async def major_angle_identification (self):
        if abs(self.angle) > MIN_STARTING_ANGLE:
            self.log.error(f'Starting angle too high: {self.angle}, adjust angle')
            await self.major_angle_correction()

    async def _timer_update(self):
        self.timer = time.time() - self.start_time

    async def _timeout_verification (self):
        if self.timer > self.timeout:
            self.log.error(f'timeout reached: {self.timer} sec')
            self.abort(*ERROR_TIMEOUT_REACHED)
            self.state = 'finish'     
###############################################################################
###############################################################################
###############################################################################
            

###############################################################################
######################## Input data and calculations ##########################
###############################################################################
            
    async def read_srf_values(self):
        start_time = time.time()
        while(True):
            timer = time.time() - start_time
            if timer > 2.0:
                self.log.error(f'failed to read SRF values for {timer} sec')
                self.abort(*ERROR_SRF_READING_FAILED)
            await asyncio.sleep(0.01)
            self.middle_srf = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_MIDDLE] * 100 
            srf_right = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_RIGHT] * 100 
            srf_left = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_LEFT] * 100
            if( math.isnan(srf_right) and not math.isnan(srf_left)):
                self.log.error('nan value recived from srf')

            if(not math.isnan(srf_right) and not math.isnan(srf_left)):

                if srf_right > MAX_SRF_VALUE:
                    srf_right = MAX_SRF_VALUE
                if srf_left > MAX_SRF_VALUE:
                    srf_left = MAX_SRF_VALUE  
                # self.middle_srf = FILTER_WEIGHT * self.dr + (1-FILTER_WEIGHT) * srf_middle  
                self.dr = FILTER_WEIGHT * self.dr + (1-FILTER_WEIGHT) * srf_right
                self.dl = FILTER_WEIGHT * self.dl + (1-FILTER_WEIGHT) * srf_left
                self.average_distance = (self.dl + self.dr)/2
                break

    async def calculate_distance_parameters(self):
        ## calculate the distance of each dl and dr (distances) and calculate the angle of oreitattion related to the normal to the cart
        if (self.dl > self.dr):
            self.sign = -1
        else:
            self.sign = 1
        
        self.delta = self.dl - self.dr
        self.angle  = math.atan2(self.delta,DISTANCE_BETWEEN_SRF_SENSORS)/math.pi * 180
        self.log.info(f'left:{self.dl}, right:{self.dr}, middle: {self.middle_srf} angle: {self.angle}')

###############################################################################
###############################################################################
###############################################################################

###############################################################################
############################ states functions #################################
###############################################################################

    async def move_backwared(self):

        kp = VELOCITY_KP
        cmd_velocity = kp*self.average_distance
        self.log.info(f'cmd_velocity {cmd_velocity}')

        if abs(cmd_velocity) > MAX_MOVING_VELOCITY:
            cmd_velocity = MAX_MOVING_VELOCITY
        if not self.obstacle_detected:
            try:
                await self.motion.set_velocity(
                            x_velocity= -1 * cmd_velocity,
                            y_velocity=0.0,
                            angular_velocity=0.0,
                            duration=2.0,
                            enable_obstacles=False,
                            wait=False,
                        )
            except Exception as error:
                self.log.error(f'linear movement failed, error: {error}')
                self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)
        else:
            await self.avoid_obstacle()
        if self.average_distance < PUSHING_IDENTIFIER_DISTANCE:
            await self.pushing_cart_identifier() 
        
    async def attach(self):
        self.log.info("stop moving, start attaching")

        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()
        try:

            while(True):
                await self.sleep(1.0)
                if(self.gripper_state['attempts'] > MAX_ATTEMPTS):
                    self.state = "finish"
                    break

                gripper_result = await self.arms.specific_robot_command(
                                        name='cart/execute',
                                        parameters={
                                                'gripper':'cart',
                                                'goal':GRIPPER_CLOSE_POSITION,
                                                'velocity':GRIPPER_VELOCITY,
                                                'pressure':GRIPPER_CLOSE_PRESSURE_CONST,
                                                'timeout':GRIPPER_TIMEOUT
                                            }, 
                                        wait=True,
                                    )
                
                self.log.debug(f'gripper result: {gripper_result}')

                await self.gripper_feedback_cb(gripper_result)
                await self.gripper_state_classifier()
    
                cart_attached = self.gripper_state['cart_attached']
                if cart_attached:
                    self.state = 'attach_verification'
                    break

                if self.gripper_state['position_reached'] == True:
                    self.log.error(f'fail, cart gripper not attached')
                    self.abort(*ERROR_GRIPPER_ATTACHMENT_FAILED)

                self.gripper_state['attempts']+=1
                if self.gripper_state['attempts'] > ATTEMPTS_BEFORE_VIBRATION:
                   await self.vibrate()


            await self.send_feedback(gripper_result)
            await self.send_feedback({'cart_attached_success' : cart_attached})

        except Exception as error:
                self.log.error(f'gripper fail error is: {error}'
                                F'error type: {type(error)}')
                self.abort(*ERROR_GRIPPER_ATTACHMENT_FAILED)
                self.state = 'finish'

    async def adjust_angle(self):
        ## Control law for minimzing the angle between the cart to the robot
        
        if (abs(self.angle) > MAX_ANGLE_STEP):
            self.angle = MAX_ANGLE_STEP * self.sign
        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()
        if not self.obstacle_detected:    
            try:
                await self.motion.rotate(
                    angle= max(abs(self.angle) * ROTATION_KP, MIN_ROTATION_ANGLE_STEP),
                    angular_speed= self.sign * ROTATING_ANGULAR_SPEED,
                    enable_obstacles=False,
                    wait=True)
            except Exception as error:
                self.log.error(f'rotation failed, error: {error}')
                self.abort(*ERROR_ROTATION_MOVEMENT_FAILED)
        else:
            await self.avoid_obstacle()

        self.log.info("finish rotate")


    async def cart_attachment_verification(self):
        self.log.info('run cart_attachment_verification')
        verification_dl=self.dl
        verification_dr=self.dr
        try:
            await self.motion.set_velocity(
                    x_velocity = VERIFICATION_VELOCITY,
                    y_velocity = 0.0,
                    angular_velocity=0.0,
                    duration=3.0,
                    enable_obstacles=False,
                    wait=False, 
                    )
            
            while (self.motion.is_moving()):
                await self.read_srf_values()
                dl_delta = abs(verification_dl - self.dl)
                dr_delta = abs(verification_dr - self.dr)
                if dl_delta < VERIFICATION_DISTANCE or dr_delta < VERIFICATION_DISTANCE:
                    self.gripper_state['cart_attached'] = True
                else:
                    self.gripper_state['cart_attached'] = False
                await asyncio.sleep(0.2)

        except Exception as error:
            self.log.error(f'linear movement failed, error: {error}')
            self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)
        self.state = 'finish'
        
###############################################################################
###############################################################################
###############################################################################

###############################################################################
####################### pre loop functions ####################################
###############################################################################
    async def pre_loop_actions(self):
        ## set the stats to diffault
        await self.set_to_diffualt()

        ### move gripper to pre-grab position

        self.pre_loop_finish = True
        try:
            gripper_result = await self.arms.specific_robot_command(
                                                    name='cart/execute',
                                                    parameters={
                                                            'gripper':'cart',
                                                            'goal':GRIPPER_OPEN_POSITION,
                                                            'velocity':GRIPPER_VELOCITY,
                                                            'pressure':GRIPPER_OPEN_PRESSURE_CONST,
                                                            'timeout':GRIPPER_TIMEOUT
                                                        }, 
                                                    wait=True,
                                                )
            
            self.log.debug(f'gripper result: {gripper_result}')

            
        except Exception as error:
            self.log.error(
                f'gripper open to pre-grab position failed, Exception type: '
                f'{type(error)}, Exception: {error}')
            self.abort(*ERROR_GRIPPER_FAILED)

        if self.rotating_180:
            await self.rotation_180()
        await self.read_srf_values()
        await self.calculate_distance_parameters()
        await self._cart_max_distance_verification()
        await self.major_angle_identification()   

        return self.pre_loop_finish

    async def rotation_180(self):
        self.log.info('Rotating 180 degree')
        try:
            await self.motion.rotate(
                angle = 180.0,
                angular_speed = ROTATING_ANGULAR_SPEED,
                enable_obstacles=False,
                wait=True)
            
        except Exception as error:
            self.log.error(f'180 rotation failed, error: {error}')
            self.abort(*ERROR_ROTATION_MOVEMENT_FAILED)

    async def set_to_diffualt(self):
        self.sign = 1
        self.state = 'idle'
        self.angle = 0
        self.dl = 0
        self.dr = 0
        self.middle_srf = 0
        self.pushing_index = 0
        self.obstacle_index = 0
        self.obstacle_detected = False
        self.obtacle_enable = True
        self.last_middle_srf = 0
        self.last_average_distance = 0.0
        self.rotating_180 = self.setup_args['180_rotating']
        self.timeout = self.setup_args['timeout']
        if self.rotating_180:
            self.timeout = self.timeout + TIMEOUT_180
        self.gripper_state = {'final_position': 0.0,
                            'final_pressure': 0.0,
                            'attempts': 0,
                            'position_reached': False,
                            'pressure_reached': False,
                            'success': False,
                            'timeout_reached': False,
                            'cart_attached': False,
                            'close_to_actual_position' : False}
        
###############################################################################
###############################################################################
###############################################################################
        
###############################################################################
##################### setup - main - finish ###################################
###############################################################################

    async def setup(self):
        self.arms = await self.enable_controller('arms')
        self.sensors = await self.enable_controller('sensors')
        self.motion = await self.enable_controller('motion')
        self.sound = await self.enable_controller('sound')
        ## create folder for audio
        create_dat_folder(AUDIO_PATH)
              


    async def main(self):

        self.log.info('SkillAttachToCart.main')

        self.start_time = time.time()
        self.timer = self.start_time

        await self.pre_loop_actions()

        while (True):

            await self._timer_update()
            await self._timeout_verification()

            ### Read the srf values and update them
            await self.read_srf_values()
            await self.calculate_distance_parameters()
            await self._cart_max_distance_verification()
            await self.obstacle_detection()
            
            ### Idetify which state you are
            await self.state_classifier()

            if self.state == 'moving':
                if REVERSE_BEEPING_ALERT:
                    if not self.sound.is_playing():
                        await self.play_predefined_sound('beep', leds = False, wait = False)
                await self.move_backwared()
            
            elif self.state == 'attaching':
                await self.attach()

            elif (self.state == 'rotating'):
                if REVERSE_BEEPING_ALERT:
                    if not self.sound.is_playing():
                        await self.play_predefined_sound('beep', leds = False, wait = False)
                await self.adjust_angle()

            elif self.state == 'attach_verification':
                await self.cart_attachment_verification()

            elif self.state == 'finish':
                await self.finish()
                
                break

            await asyncio.sleep(0.2)

    async def finish(self):
        cart_attached = self.gripper_state['cart_attached']
        self.log.info(f'cart attachment status is: {cart_attached}, time to execute: {self.timer}')
        await self.send_feedback('application finished, cart attachment is: '\
                    f'{cart_attached}')
        # if self.gripper_state['cart_attached'] is False:
        #     self.abort(*ERROR_CART_NOT_ATTACHED)
        self.log.info('SkillAttachToCart.finish')
        # await self.skill_apr2cart.execute_finish()
    
###############################################################################
###############################################################################
###############################################################################
        
###############################################################################
#########################       UI           ##################################
###############################################################################        

    async def play_predefined_sound(self,
                                    recording_name,
                                    audio_type = 'mp3',
                                    leds = False,
                                    wait = True):
            '''
            INPUTS:
                recording_name - the name of the recording ; str
            
            OUTPUTS:
                This function has no outputs. It plays a sound
            '''

            path = f'{AUDIO_PATH}/{recording_name}.{audio_type}'

            if audio_type == 'mp3': # TODO: Add something more robust than eyed3 package
                self.audio_duration = eyed3.load(resolve_path(path)).info.time_secs

            try:
                await self.sound.play_sound(
                    path = f'{AUDIO_PATH}/{recording_name}.{audio_type}',
                    wait = wait,
                    callback_feedback = self.cb_feedback_sound,
                    callback_finish = self.cb_finish_sound,
                    volume = SOUND_VOLUME
                    )

            except Exception as e:
                self.log.warning(f'Skipped playing sound, got error {e}')  

###############################################################################
###############################################################################
###############################################################################

    


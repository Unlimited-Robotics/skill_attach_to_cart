from raya.skills import RayaSkill
from raya.controllers.robot_skills_controller import RobotSkillsController
from raya.controllers.motion_controller import MotionController
from raya.controllers.sensors_controller import SensorsController
from raya.exceptions import RayaMotionException
import math
import time
import asyncio
from .constants import *


class SkillDetachFromCart(RayaSkill):

    DEFAULT_SETUP_ARGS = {
        'debug_level': DEFAULT_DEBUG_LEVEL
    }

    REQUIRED_SETUP_ARGS = {}

    DEFAULT_EXECUTE_ARGS = {
        'gripper_tries': 3,
    }


###############################################################################
##################### setup - main - finish ###################################
###############################################################################

    async def setup(self):
        self.debug_level = self.setup_args['debug_level']
        self.show_debug('SkillDetachFromCart.setup')
        self.robot_skills: RobotSkillsController = \
            await self.enable_controller('robot_skills')
        self.motion: MotionController = \
            await self.enable_controller('motion')
        self.sensors: SensorsController = \
            await self.enable_controller('sensors')


    async def main(self):
        self.show_debug('SkillDetachFromCart.main')
        self.start_time = time.time()
        self.timer = self.start_time
        await self._set_to_default()

        try:

            for _ in range(self.execute_args['gripper_tries']):
                self.show_debug('Detaching cart...')

                gripper_result = await self.robot_skills.execute_skill(
                                skill='cart_gripper_execute',
                                    hand='cart',
                                    goal= GRIPPER_CLOSE_POSITION,
                                    velocity=GRIPPER_VELOCITY,
                                    pressure= GRIPPER_OPEN_PRESSURE_CONST,
                                    timeout=GRIPPER_TIMEOUT,
                                wait=True,  
                            )
                

                self.show_debug(f'gripper result: {gripper_result}')
                await self._gripper_feedback_cb(*gripper_result)
                await self._gripper_state_classifier()
    
                cart_attached = self.gripper_state['cart_attached']

                if not cart_attached:
                    break

                await self._timer_update()
                await self._timeout_verification()
                await self._read_srf_values()

        except Exception as error:
                self.show_debug((
                    f'gripper fail error is: {error}'
                    f'error type: {type(error)}'
                ))
                self.error_type = ERROR_GRIPPER_ATTACHMENT_FAILED
                await self.finish()
                
        await self._cart_detachment_verification()
        await self.finish()

        


    async def finish(self):
        is_moving = self.motion.is_moving()
        if (is_moving):
            await self.motion.cancel_motion()

        cart_attached = self.gripper_state['cart_attached']
        
        self.show_debug(
            f'cart attachment status is: {cart_attached}, '
            f'time to execute: {self.timer}'
        )
        await self.send_feedback(cart_attached)

        if self.gripper_state['cart_attached'] is True and \
                self.error_type is None:
            self.show_debug('Cart still attached')
            self.error_type = ERROR_CART_STILL_ATTACHED
        if self.error_type != None:
            self.abort(*self.error_type)
        else:
            self.show_debug('Cart detached successfully')
        self.show_debug('SkillAttachToCart.finish')

 #############################################################################
 ############################### helper functions ############################
 ############################################################################
            
    def show_debug(self, msg: str = ''):
        if self.debug_level:
            self.log.debug(msg)

    
    async def _gripper_state_classifier(self):
        if (self.gripper_state['result_code'] == TOUCHING_OBJECT_CODE):
            self.gripper_state['cart_attached'] = True

        elif (self.gripper_state['result_code'] == REACHED_CLOSE_POSE_CODE):
            self.gripper_state['cart_attached'] = False

    async def _cart_detachment_verification(self):
        self.show_debug('run _cart_detachment_verification')
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
                await self._read_srf_values()
                await asyncio.sleep(0.2)

                dl_delta = abs(verification_dl - self.dl)
                dr_delta = abs(verification_dr - self.dr)

                if dl_delta < DETACH_VERIFICATION_DISTANCE and \
                        dr_delta < DETACH_VERIFICATION_DISTANCE:
                    self.gripper_state['cart_attached'] = True
                    self.show_debug(f'Cart is still attached dl_delta: {dl_delta},\
                                     dr_delta: {dr_delta}, \
                                    VERIFICATION_DISTANCE: {DETACH_VERIFICATION_DISTANCE}')
                    self.error_type = ERROR_CART_STILL_ATTACHED
                else:
                    self.show_debug(f'Cart not attached dl_delta: {dl_delta},\
                                     dr_delta: {dr_delta}, \
                                    VERIFICATION_DISTANCE: {DETACH_VERIFICATION_DISTANCE}')
                    self.gripper_state['cart_attached'] = False
                    self.error_type = None

        except Exception as error:
            self.show_debug(f'linear movement failed, error: {error}')
            self.error_type(ERROR_LINEAR_MOVEMENT_FAILED)
            await self.finish()
        

        
    async def _gripper_feedback_cb(self, success,\
                                   result_code,\
                                      result_str,\
                                          final_position,\
                                              final_pressure):
        
        ## INPUT: gripper feednack result from raya
        ## function updates local list of 
        ##parameters which define the gripper state

        self.gripper_state['result_code'] = result_code
        self.gripper_state['final_position'] =  final_position
        self.gripper_state['final_pressure'] = final_pressure
        self.gripper_state['success'] = success
        self.show_debug(f'gripper_state: {self.gripper_state}')

               
    async def _timer_update(self):
        self.timer = time.time() - self.start_time


    async def _timeout_verification (self):
        if self.timer > DETACH_TIMEOUT:
            self.show_debug(f'timeout reached: {self.timer} sec')
            self.error_type = ERROR_TIMEOUT_REACHED
            await self.finish()


    async def _read_srf_values(self):
        ## read srf value with the index, the srf of the cart is 5 and 2
        start_time = time.time()
        while(True):
            timer = time.time() - start_time
            if timer > 2.0:
                self.show_debug(f'failed to read SRF values for {timer} sec')
                self.error_type = ERROR_SRF_READING_FAILED
                await self.finish()
            await asyncio.sleep(0.01)
            srf_right = self.sensors.get_sensor_value(
                'srf')[SRF_SENSOR_ID_RIGHT]*SRF_M2CM
            srf_left = self.sensors.get_sensor_value(
                'srf')[SRF_SENSOR_ID_LEFT]*SRF_M2CM
            if( math.isnan(srf_right) and not math.isnan(srf_left)):
                self.show_debug('nan value recived from srf')

            if(not math.isnan(srf_right) and not math.isnan(srf_left)):

                if srf_right > MAX_SRF_VALUE:
                    srf_right = MAX_SRF_VALUE
                if srf_left > MAX_SRF_VALUE:
                    srf_left = MAX_SRF_VALUE  
                self.dr = FILTER_WEIGHT * self.dr + \
                    (1-FILTER_WEIGHT) * srf_right
                self.dl = FILTER_WEIGHT * self.dl + \
                    (1-FILTER_WEIGHT) * srf_left
                break


    async def _set_to_default(self):
        self.dl = 0
        self.dr = 0
        self.gripper_state = {
            'final_position': 0.0,
            'final_pressure': 0.0,
            'position_reached': False,
            'pressure_reached': False,
            'success': False,
            'timeout_reached': False,
            'cart_attached': True,
        }
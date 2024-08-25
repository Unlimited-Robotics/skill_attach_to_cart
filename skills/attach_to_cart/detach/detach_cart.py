import asyncio
import math
import time

from raya.skills import RayaSkill
from raya.controllers.arms_controller import ArmsController
from raya.controllers.motion_controller import MotionController
from raya.controllers.sensors_controller import SensorsController
from raya.controllers.robot_skills_controller import RobotSkillsController
from .constants import *


class SkillDetachCart(RayaSkill):


    DEFAULT_SETUP_ARGS = {

        'timeout' : FULL_APP_TIMEOUT,

    }
    REQUIRED_SETUP_ARGS = {

    }
    
    DEFAULT_EXECUTE_ARGS = {}

    REQUIREDT_EXECUTE_ARGS = {}


    async def calculate_distance_parameters(self):
        ## calculate the distance of each dl and dr (distances) and 
        ## calculate the angle of oreitattion related to the normal to the cart
        if (self.dl > self.dr):
            self.sign = -1
        else:
            self.sign = 1

        self.delta = self.dl - self.dr
        self.angle  = math.atan2(
            self.delta,DISTANCE_BETWEEN_SRF_SENSORS)/math.pi * 180
        self.average_distance = (self.dl + self.dr)/2



    async def gripper_state_classifier(self):
        if (self.gripper_state['result_code'] == 3):
            self.gripper_state['cart_attached'] = True

        elif (self.gripper_state['result_code'] == 1):
            self.gripper_state['cart_attached'] = False

    async def cart_detachment_verification(self):
        self.log.info('run cart_detachment_verification')
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
                await asyncio.sleep(0.2)
                await self.read_srf_values()
                dl_delta = abs(verification_dl - self.dl)
                dr_delta = abs(verification_dr - self.dr)

                if dl_delta < VERIFICATION_DISTANCE or \
                        dr_delta < VERIFICATION_DISTANCE:
                    self.gripper_state['cart_attached'] = True
                else:
                    self.gripper_state['cart_attached'] = False

        except Exception as error:
            self.log.error(f'linear movement failed, error: {error}')
            self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)
        self.state = 'finish'


    async def vibrate(self):
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


    async def detach(self):
        self.log.info("detaching cart")

        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()
        try:

            while(True):
                await self.sleep(1.0)
                if(self.gripper_state['attempts'] > MAX_ATTEMPTS):
                    break

                gripper_result = await self.robot_skills.execute_skill(
                                skill='cart_gripper_execute',
                                    hand='cart',
                                    goal= GRIPPER_CLOSE_POSITION,
                                    velocity=GRIPPER_VELOCITY,
                                    pressure= GRIPPER_OPEN_PRESSURE_CONST,
                                    timeout=GRIPPER_TIMEOUT,
                                wait=True,  
                            )
                

                self.log.debug(f'gripper result: {gripper_result}')
                await self.gripper_feedback_cb(gripper_result)

                await self.gripper_state_classifier()
    
                cart_attached = self.gripper_state['cart_attached']

                if not cart_attached:
                    break
                    
                    
                self.gripper_state['attempts']+=1
                if self.gripper_state['attempts'] > ATTEMPTS_BEFORE_VIBRATION:
                   await self.vibrate()

                await self._timer_update()
                await self._timeout_verification()
                await self.read_srf_values()
                await self.calculate_distance_parameters()

            await self.send_feedback(gripper_result)
            await self.send_feedback({
                'cart_detached_success' : not cart_attached
            })
            await self.cart_detachment_verification()

        except Exception as error:
                self.log.error((
                    f'gripper fail error is: {error}'
                    f'error type: {type(error)}'
                ))
                self.abort(*ERROR_GRIPPER_DETACHMENT_FAILED)
                self.state = 'finish'


    async def gripper_feedback_cb(self, success, result_code, result_str, final_position, final_pressure):
        
        ## INPUT: gripper feednack result from raya
        ## function updates local list of parameters which define the gripper state
        self.gripper_state['result_code'] = result_code
        self.gripper_state['final_position'] =  final_position
        self.gripper_state['final_pressure'] = final_pressure
        self.gripper_state['success'] = success
        self.log.debug(f'gripper_state: {self.gripper_state}')

               
    async def _timer_update(self):
        self.timer = time.time() - self.start_time


    async def _timeout_verification (self):
        if self.timer > self.timeout:
            self.log.error(f'timeout reached: {self.timer} sec')
            self.abort(*ERROR_TIMEOUT_REACHED)
            self.state = 'finish'


    async def read_srf_values(self):
        ## read srf value with the index, the srf of the cart is 5 and 2
        start_time = time.time()
        while(True):
            timer = time.time() - start_time
            if timer > 2.0:
                self.log.error(f'failed to read SRF values for {timer} sec')
                self.error_type = ERROR_SRF_READING_FAILED
                self.state = 'finish'
            await asyncio.sleep(0.01)
            srf_right = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_RIGHT]*SRF_M2CM
            srf_left = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_LEFT]*SRF_M2CM
            if( math.isnan(srf_right) and not math.isnan(srf_left)):
                self.log.error('nan value recived from srf')

            if(not math.isnan(srf_right) and not math.isnan(srf_left)):

                if srf_right > MAX_SRF_VALUE:
                    srf_right = MAX_SRF_VALUE
                if srf_left > MAX_SRF_VALUE:
                    srf_left = MAX_SRF_VALUE  
                self.dr = FILTER_WEIGHT * self.dr + (1-FILTER_WEIGHT) * srf_right
                self.dl = FILTER_WEIGHT * self.dl + (1-FILTER_WEIGHT) * srf_left
                break


    async def set_to_default(self):
        self.sign = 1
        self.state = 'idle'
        self.angle = 0
        self.dl = 0
        self.dr = 0
        self.timeout = self.setup_args['timeout']
        self.gripper_state = {
            'final_position': 0.0,
            'final_pressure': 0.0,
            'attempts': 0,
            'position_reached': False,
            'pressure_reached': False,
            'success': False,
            'timeout_reached': False,
            'cart_attached': True,
            'close_to_actual_position' : False
        }


    async def setup(self):
        # self.arms:ArmsController = \
        #     await self.enable_controller('arms')
        self.sensors:SensorsController = \
            await self.enable_controller('sensors')
        self.motion:MotionController = \
            await self.enable_controller('motion')
        self.robot_skills:RobotSkillsController = \
            await self.enable_controller('robot_skills')


    async def main(self):
        ### approach state

        self.log.info('SkillDetachCart.main')
        await self.set_to_default()

        self.start_time = time.time()
        self.timer = self.start_time
        await self._timer_update()
        await self._timeout_verification()

        await self.read_srf_values()

        await self.calculate_distance_parameters()
        
        await self.detach()

        await self.finish()
        

    async def finish(self):
        cart_attached = self.gripper_state['cart_attached']
        self.log.info((
            f'cart detachment status is: {not cart_attached}, '
            f'time to execute: {self.timer}'
        ))
        await self.send_feedback((
            'application finished, cart detachment is: '
            f'{not cart_attached}'
        ))
        self.log.info('SkillDetachCart.finish')

import time

from raya.skills import RayaSkill
from raya.controllers.robot_skills_controller import RobotSkillsController
from raya.controllers.motion_controller import MotionController

from .constants import *


class SkillDetachFromCart(RayaSkill):

    DEFAULT_SETUP_ARGS = {}
    
    REQUIRED_SETUP_ARGS = {}
    
###############################################################################
##################### setup - main - finish ###################################
###############################################################################

    async def setup(self):
        self.log.debug('SkillDetachFromCart.setup')
        self.robot_skills : RobotSkillsController = \
            await self.enable_controller('robot_skills')
        self.motion : MotionController = \
            await self.enable_controller('motion')


    async def main(self):
        self.log.debug('SkillDetachFromCart.main')
        
        self.log.debug('Opening gripper...')
        self.gripper_running = True
        await self.robot_skills.execute_skill(
            skill='cart_gripper_execute',
            hand='cart',
            goal= GRIPPER_CLOSE_POSITION,
            velocity=GRIPPER_VELOCITY,
            pressure= GRIPPER_OPEN_PRESSURE_CONST,
            timeout=GRIPPER_TIMEOUT,
            wait=False,
            callback_feedback_async=self.__cb_gripper_feedback,
            callback_finish_async=self.__cb_gripper_finish
        )
        while self.gripper_running:
            await self.sleep(1.0)
        
        self.log.debug('Moving Forwards...')
        self.running_motion = True
        await self.motion.set_velocity(
            x_velocity = VERIFICATION_VELOCITY,
            y_velocity = 0.0,
            angular_velocity = 0.0,
            duration = DURATION_DETTACHING,
            enable_obstacles = True,
            wait = False,
            callback_feedback_async = self._cb_motion_feedback,
            callback_finish_async = self._cb_motion_finish
        )
        
        while self.running_motion:
            await self.sleep(1.0)


    async def finish(self):
        self.log.debug('SkillDetachFromCart.finish')


###############################################################################
##################### CALLBACKS ###############################################
###############################################################################

    async def __cb_gripper_feedback(self, 
            actual_position,
            actual_velocity,
            actual_pressure,
            actual_fsr_right,
            actual_fsr_left,
            time_left
        ):
        self.log.debug('---------------------------------')
        self.log.debug(f'__cb_gripper_feedback:')
        self.log.debug(f'actual_position: {actual_position}')
        self.log.debug(f'actual_velocity: {actual_velocity}')
        self.log.debug(f'actual_pressure: {actual_pressure}')
        self.log.debug(f'actual_fsr_right: {actual_fsr_right}')
        self.log.debug(f'actual_fsr_left: {actual_fsr_left}')
        self.log.debug(f'time_left: {time_left}')
        self.log.debug('---------------------------------')
        


    async def __cb_gripper_finish(self, 
            success,
            result_code,
            result_str,
            final_position,
            final_pressure
        ):
        self.log.debug('---------------------------------')
        self.log.debug(f'__cb_gripper_finish:')
        self.log.debug(f'success: {success}')
        self.log.debug(f'result_code: {result_code}')
        self.log.debug(f'result_str: {result_str}')
        self.log.debug(f'final_position: {final_position}')
        self.log.debug(f'final_pressure: {final_pressure}')
        self.log.debug('---------------------------------')
        self.gripper_running = False


    async def _cb_motion_feedback(self, code , feedback):
        self.log.debug('---------------------------------')
        self.log.debug('_cb_motion_feedback:')
        self.log.debug(f'code {code}, feedback: {feedback}')
        self.log.debug('---------------------------------')

        
    async def _cb_motion_finish(self, code, result):
        self.log.debug('---------------------------------')
        self.log.debug('_cb_motion_finish:')
        self.log.debug(f'code {code}, finish: {result}')
        self.log.debug('---------------------------------')
        self.running_motion = False
        

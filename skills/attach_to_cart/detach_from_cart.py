from raya.skills import RayaSkill
from raya.controllers.robot_skills_controller import RobotSkillsController
from raya.controllers.motion_controller import MotionController

from raya.exceptions import RayaMotionException

from .constants import *


class SkillDetachFromCart(RayaSkill):

    DEFAULT_SETUP_ARGS = {}
    
    REQUIRED_SETUP_ARGS = {}
    
    DEFAULT_EXECUTE_ARGS = {
        'gripper_tries': 3,
    }

    
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
        
        for _ in range(self.execute_args['gripper_tries']):
            self.log.debug('Detaching cart...')
            try:
                gripper_result = await self.robot_skills.execute_skill(
                    skill='cart_gripper_execute',
                    hand='cart',
                    goal= GRIPPER_CLOSE_POSITION,
                    velocity=GRIPPER_VELOCITY,
                    pressure= GRIPPER_OPEN_PRESSURE_CONST,
                    timeout=GRIPPER_TIMEOUT,
                    wait=True
                )
                self.log.debug(f'Gripper result: {gripper_result}')
                if gripper_result[0] == 0:
                    self.log.debug('Cart detached successfully')
                    break
                else:
                    self.log.debug('Detach Failed, retrying...')
            except Exception as e:
                self.log.error(f'Error opening gripper: {e}')
        
        self.log.debug('Moving Forwards...')
        try:
            await self.motion.set_velocity(
                x_velocity = VERIFICATION_VELOCITY,
                y_velocity = 0.0,
                angular_velocity = 0.0,
                duration = DURATION_DETTACHING,
                enable_obstacles = True,
                wait = True
            )
        except RayaMotionException as e:
            self.log.debug(f'Error moving forwards: {type(e)}')
        else:
            self.log.debug('Moved Forwards')


    async def finish(self):
        self.log.debug('SkillDetachFromCart.finish')

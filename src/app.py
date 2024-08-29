from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart
from skills.attach_to_cart import SkillDetachFromCart


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info(f'RayaApplication.setup')
        if self.mode == 'attach':
            self.skill_att2cart = self.register_skill(SkillAttachToCart)
            await self.skill_att2cart.execute_setup(
                setup_args={
                    'reverse_beeping_alert': False
                },
            )
        elif self.mode == 'detach':
            self.skill_deatt2cart = self.register_skill(SkillDetachFromCart)
            await self.skill_deatt2cart.execute_setup(
                setup_args={},
            )
        self.log.info(f'Skill {self.mode} registered')


    async def cb_skill_done(self, exception, result):
        self.log.info(f'cb_skill_done, cart {self.mode}, result: {result}')
        if exception is None:
            if self.mode == 'attach':
                await self.skill_att2cart.execute_finish()
            elif self.mode == 'detach':
                await self.skill_deatt2cart.execute_finish()
        else:
            self.log.warn(
                f'error occured while {self.mode}ing, exception type: '
                f'{type(exception)} {exception}'
            )


    async def cb_skill_feedback(self, feedback):
        self.log.info(feedback)


    def get_arguments(self):
        self.mode = self.get_argument(
            '-m', '--mode',
            type=str,
            help='Run mode, attach or detach',
            required=True
        )
        if self.mode not in ['attach', 'detach']:
            raise ValueError('Mode should be attach or detach')


    async def main(self):
        if self.mode == 'attach':
            await self.skill_att2cart.execute_main(
                wait=True
            )
        elif self.mode == 'detach':
            await self.skill_deatt2cart.execute_main(
                wait=True
            )


    async def finish(self):
        self.log.info(f'RayaApplication.finish')

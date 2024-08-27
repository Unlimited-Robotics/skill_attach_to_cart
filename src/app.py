from raya.application_base import RayaApplicationBase
from raya.exceptions import RayaSkillAborted
from raya.enumerations import SKILL_STATE

from skills.attach_to_cart import SkillAttachCart, SkillDetachCart


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info('RayaApplication.setup')

        self.skill_att2cart = self.register_skill(SkillAttachCart)
        self.skill_detach = self.register_skill(SkillDetachCart)
        
        if self.attach:
            await self.skill_att2cart.execute_setup(
                setup_args={},
            )
            
            execute_args = {}
            await self.skill_att2cart.execute_main(
                execute_args=execute_args,
                callback_done=self.cb_skill_done,
                callback_feedback=self.cb_skill_feedback,
                wait=False
            )
        elif self.detach:
            await self.skill_detach.execute_setup(
                setup_args={}
            )
            
            execute_args = {}
            await self.skill_detach.execute_main(
                execute_args=execute_args,
                callback_done=self.cb_skill_done,
                callback_feedback=self.cb_skill_feedback,
                wait=False
            )
        self.log.info('RayaApplication.setup done')


    async def loop(self):
        try:
            if self.skill_att2cart.get_execution_state() == SKILL_STATE.EXECUTED:
                await self.skill_att2cart.raise_last_execution_exception()
            elif self.skill_detach.get_execution_state() == SKILL_STATE.EXECUTED:
                await self.skill_detach.raise_last_execution_exception()
        except RayaSkillAborted as e:
            self.log.error(
                f'Skill aborted with error_code: {e.error_code}, '
                f'error_msg: {e.error_msg}'
            )
        else:
            for _ in range(10):
                self.log.debug('RayaApplication.loop')
                await self.sleep(1.0)
            self.finish_app()
        
        await self.sleep(1.0)


    async def finish(self):
        self.log.info('RayaApplication.finish')
        if self.skill_att2cart.get_execution_state() == SKILL_STATE.EXECUTED:
            self.log.info('finishing att2cart')
            res = await self.skill_att2cart.execute_finish()
            self.log.info(f'att2cart finish result: {res}')

        if self.skill_detach.get_execution_state() == SKILL_STATE.EXECUTED:
            self.log.info('finishing detach')
            res = await self.skill_detach.execute_finish()
            self.log.info(f'detach finish result: {res}')
        
        self.log.info('RayaApplication.finish done')



    async def cb_skill_done(self, exception, result):
        self.log.debug(
            f'cb_skill_done, exception: {exception} result: {result}'
        )


    async def cb_skill_feedback(self, feedback):
        self.log.info(feedback)


    def get_arguments(self):
        self.attach = self.get_flag_argument(
            '-a', '--attach',
            help='Execute attach to cart',
        )
        
        self.detach = self.get_flag_argument(
            '-d', '--detach',
            help='Execute detach to cart',
        )
        
        if not self.attach and not self.detach:
            raise ValueError(
                'No flag provided, please provide either --attach or --detach'
            )
        if self.attach and self.detach:
            raise ValueError(
                'Both flags provided, please provide only one option'
            )

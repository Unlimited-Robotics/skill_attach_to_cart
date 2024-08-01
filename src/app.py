from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart, SkillDetachCart
from raya.exceptions import RayaSkillAborted


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info(f'RayaApplication.setup')

        self.skill_att2cart = self.register_skill(SkillAttachToCart)
        self.skill_detach = self.register_skill(SkillDetachCart)
        if self.attach:
            await self.skill_att2cart.execute_setup({})
            self.target_tags = [str(int(tag)) for tag in self.target_tags]
            self.log.debug(f'target_tags: {self.target_tags}')
            self.tag_size = self.tag_size[0]
        elif self.detach:
            await self.skill_detach.execute_setup({})
            
        
        # Correct bad parser arguments
        if self.attach:
            exectute_args = {
                'target_distance': self.target_distance,
                'reverse': self.reverse,
                'tag_size': self.tag_size,
                'target_tags': self.target_tags
            }
            
            await self.skill_att2cart.execute_main(
                execute_args=exectute_args,
                callback_done=self.cb_skill_done,
                callback_feedback=self.cb_skill_feedback,
                wait=False
            )
        elif self.detach:
            exectute_args = {}
            await self.skill_detach.execute_main(
                execute_args=exectute_args,
                callback_done=self.cb_skill_done,
                callback_feedback=self.cb_skill_feedback,
                wait=False
            )

    async def loop(self):
        if self.attach:
            try:
                result = await self.skill_att2cart.wait_main()
                self.log.info(f'att2cart result: {result}')
                
                self.log.info('finishing att2cart')
                res = await self.skill_att2cart.execute_finish()
                self.log.info(f'att2cart finish result: {res}')
            except RayaSkillAborted as e:
                self.log.error(f'Skill aborted with error_code: {e.error_code}, error_msg: {e.error_msg}')
            
        elif self.detach:
            try:
                result = await self.skill_detach.wait_main()
                self.log.info(f'detach result: {result}')

                self.log.info('finishing detach')
                res = await self.skill_att2cart.execute_finish()
                self.log.info(f'detach finish result: {res}')
            except RayaSkillAborted as e:
                self.log.error(f'Skill aborted with error_code: {e.error_code}, error_msg: {e.error_msg}')
            
        while True:
            await self.sleep(1)
            self.log.debug(f'RayaApplication.loop')


    async def finish(self):
        self.log.info(f'RayaApplication.finish')


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
                'Both flags provided, please provide only one flag'
            )
        
        if self.attach:
            self.target_distance = self.get_argument(
                '-td', '--target_distance',
                type=float,
                help='Distance to target',
                default=0.7
            )
            
            self.reverse = self.get_flag_argument(
                '-r', '--reverse',
                help='Reverse the robot',
            )
            
            self.tag_size = self.get_argument(
                '-ts', '--tag_size',
                type=float,
                help='Size of the tag in meters',
                nargs='+',
                required=True
            )
            
            self.target_tags = self.get_argument(
                '-tt', '--target_tags',
                type=float,
                help='Tags to target',
                nargs='+',
                required=True
            )
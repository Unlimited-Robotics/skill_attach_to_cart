from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart, SkillDetachCart


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info(f'RayaApplication.setup')

        self.skill_att2cart = self.register_skill(SkillAttachToCart)
        self.skill_detach = self.register_skill(SkillDetachCart)
        if self.attach:
            await self.skill_att2cart.execute_setup({})
        elif self.detach:
            await self.skill_detach.execute_setup({})
        
        if self.attach:
            exectute_args = {
                'target_distance': self.target_distance,
                'reverse': self.reverse,
                'tag_size': self.tag_size,
                'target_tags': self.target_tags
            }
            
            await self.skill_att2cart.execute_main(
                execute_args={},
                callback=self.cb_skill_done,
                feedback_callback=self.cb_skill_feedback,
                wait=False
            )
        elif self.detach:
            await self.skill_detach.execute_main(
                execute_args={},
                callback=self.cb_skill_done,
                feedback_callback=self.cb_skill_feedback,
                wait=False
            )
        

    async def main(self):
        if self.attach:
            result = await self.skill_att2cart.wait_main()
            self.log.info(f'att2cart result: {result}')
        elif self.detach:
            result = await self.skill_detach.wait_main()
            self.log.info(f'detach result: {result}')
            


    async def finish(self):
        self.log.info(f'RayaApplication.finish')


    async def cb_skill_done(self, exception, result):
        # self.log.info(f'cb_skill_done!!!!! exception: {type(exception)}')
        self.log.info(f'cb_skill_done, result: {result}')
        if exception is None:
            await self.skill_att2cart.execute_finish()
        else: 
            self.log.warn(
                    'error occured while attaching, exception type: '
                    f'{type(exception)} {exception}'
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
                type=list,
                help='Size of the tag in meters',
                nargs='+',
                required=True
            )
            
            self.target_tags = self.get_argument(
                '-tt', '--target_tags',
                type=list,
                help='Tags to target',
                nargs='+',
                required=True
            )
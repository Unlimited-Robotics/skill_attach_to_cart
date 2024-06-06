# Ra-Ya Skill - Attach to Cart

[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://github.com/Unlimited-Robotics/skill_attach_to_cart/graphs/commit-activity)

## Description

This skill is used when gary required to attach to cart, Attach to cart skill uses approach to tag skill inside the logic

## Requirements

* [Ra-Ya controllers]: MotionController, CVController

Required Hardware:
1. cart gripper
2. SRF sensors connected to the cart gripper
3. cart with adapter
default parameters:
1. 'timeout' - full skill timeout 70 sec as default
2. '180_rotating' - bool for include 180 degree rotate, false as default
3. 'actual_desired_position' - parameter for close pistion identification
4. 'reverse_beeping_alert' - bool false as defualt
5. 'close_pressure' - pressure const, default as 0.6



## Basic logic
1. Approach to tag
2. Rotate 180 degrees
3. State machine which runs on three posible states: \
    3.1 linear velocity backwards \
    3.2 adjust angle \
    3.3 attach to cart
4. After attaching gary run cart attach verification with moving forward and riding back SRF values to identify if the cart is connected

All constant parameters can be changed inside constants.py file.


## Installation

``` bash
rayasdk skills install attach_to_cart
```

## Usage

This example will approach to the tags with identifier 1 and 2 both have a size of 8 cm, it will rotate 180 degrees and then it will attach to cart.

``` python
from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart
from raya.exceptions import *


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info(f'RayaApplication.setup')
        self.skill_aborted = False
        self.skill_att2cart = self.register_skill(SkillAttachToCart)
        try:
            await self.skill_att2cart.execute_setup(
                    setup_args={
                            'tags_size': 0.08,
                        },
                )
        except RayaSkillAborted as error:
            self.log.info(f'error: {error},type: {type(error)}')


    async def main(self):
        try:
            await self.skill_att2cart.execute_main(
                execute_args={
                    'identifier': [1 , 2]
                }
            )
        except RayaSkillAborted as error:
            self.skill_aborted = True
            self.log.error(f'error code: {error.error_code}, error: {error.error_msg}')
            self.log.warn('cart NOT connected')


    async def finish(self):
        if not self.skill_aborted:
            await self.skill_att2cart.execute_finish()
        self.log.info(f'RayaApplication.finish')


    async def cb_skill_feedback(self, feedback):
        self.log.info(feedback)

```

## Exceptions

| Exception | Value (error_code, error_msg) |
| :-------  | :--- |
| ERROR_CART_NOT_ACCESSABLE | (0, 'cart is too far or not accessable') |
| ERROR_TIMEOUT_REACHED | (1,'loop timeout reached') |
| ERROR_CART_NOT_ATTACHED | (2, 'cart was not attached') |
| ERROR_GRIPPER_ATTACHMENT_FAILED | (3, 'gripper attachment failed') |
| ERROR_APPROACH_FAILED | (4, 'approach to tag failed') |
| ERROR_LINEAR_MOVEMENT_FAILED | (5, 'gary linear motion command failed') |
| ERROR_ROTATION_MOVEMENT_FAILED | (6, 'rotation motion command failed') |
| ERROR_GRIPPER_FAILED | (7, 'gripper opening failed') |
| ERROR_CART_NOT_GETTING_CLOSER | (8, 'attach failed, cart pushed') |
| ERROR_SENSOR_NOISE | (9, 'attach failed, IR sensor recive too much noise') |

## Arguments

### Setup

#### Required

| Name              | Type     | Description |
| :--------------- | :------: | :---- |
| tags_size         | float    | Size of the tags to use, in meters, all tags have to same the same size. |

#### Default

| Name          | Type | Default value | Description |
| :---------------- | :------: | :------: | :---- |
| fsm_log_transitions | boolean | True | Shows the log of the transitions of the fsm. |
| enable_obstacles | boolean | True | Enables the obstacle detection when the robot moves, in case of obstacle it tries to avoid it, If the obstacles does not moves it will raise and exception |

### Execute

#### Required

| Name          | Type | Description |
| :---------------- | :------: | :---- |
| identifier | [int] | List of identifiers of the tags to use, The order of this list must be the same as the order of the tags. Like this: [tag1, tag2, tag3] means that the tag1 is the one on the left, tag2 is the one in the middle and tag3 is the one on the right. |

#### Default


| Name                              | Type    | Default value | Description                                                                                       |
| :---------------------------     | :----:  | :-----------: | :------------------------------------------------------------------------------------------------  |
| distance_to_goal                  | float   | 0.1           | Distance to the goal, in meters; the robot stops upon reaching this distance to the goal and corrects the angle.         |
| angle_to_goal                     | float   | 0.0           | Angle in degrees; approximation angle to target.                                                  |
| angular_velocity                  | int     | 10            | Angular velocity.                                                                                 |
| linear_velocity                   | float   | 0.1           | Linear velocity.                                                                                  |
| min_correction_distance           | float   | 0.5           | Minimum correction distance.                                                                      |
| max_misalignment                  | float   | 1.0           | Maximum misalignment.                                                                             |
| step_size                         | float   | 0.2           | Step size.                                                                                        |
| tags_to_average                   | int     | 6             | Number of tags to average.                                                                        |
| max_x_error_allowed               | float   | 0.02          | Maximum allowed error in the x-axis.                                                              |
| max_y_error_allowed               | float   | 0.05          | Maximum allowed error in the y-axis.                                                              |
| max_angle_error_allowed           | float   | 5.0           | Maximum allowed error in angle.                                                                   |
| allowed_motion_tries              | int     | 10            | Number of allowed motion tries.                                                                   |
| max_allowed_distance              | float   | 2.5           | Maximum allowed distance from target.                                                             |
| scaling_step_to_checking          | float   | 1.5           | Scaling step to checking.                                                                         |
| max_allowed_rotation              | int     | 40            | Maximum rotation allowed in degrees.                                                              |
| min_allowed_rotation_intersection | float   | 5.0           | Minimum allowed rotation at the intersection in degrees.                                          |
| max_reverse_adjust                | float   | 0.2           | Maximum reverse adjustment to correct final angle in meters.                                      |
| max_allowed_correction_tries      | int     | 3             | Maximum allowed correction tries in final correction.                                             |
| enable_initial_reverse_adjust     | boolean | False         | Enable initial reverse adjustment.                                                                |
| enable_final_reverse_adjust       | boolean | False         | Enable final reverse adjustment.                                                                  |
| enable_step_intersection          | boolean | False         | Enable step intersection.                                                                         |
| correct_if_only_one_tag           | boolean | False         | In case that it detects only one tag it will rotate `max_angle_if_oy_one_tag` to see the other tag. |
| max_angle_if_oy_one_tag           | int     | 30            | Maximum angle if `correct_if_only_one_tag` is set to true.                                        |
| y_offset                          | float   | 0.0           | Y-axis offset.                                                                                    |
 





###### ERRORS

ERROR_SENSOR_NOISE = (9,'attach failed, IR sensor recive too much noise')
ERROR_CART_NOT_GETTING_CLOSER = (8,'attach failed, cart pushed')
ERROR_GRIPPER_FAILED = (7,'gripper opening failed')
ERROR_ROTATION_MOVEMENT_FAILED = (6,'rotation motion command failed')
ERROR_LINEAR_MOVEMENT_FAILED = (5,
                'gary linear motion command failed')
ERROR_APPROACH_FAILED = (4,

                'approach to tag failed')
ERROR_GRIPPER_ATTACHMENT_FAILED = (3,'gripper attachment failed')
ERROR_CART_NOT_ATTACHED = (2,'cart was not attached')
ERROR_TIMEOUT_REACHED = (1,f'loop timeout reached')
ERROR_CART_NOT_ACCESSABLE = (0, 'cart is too far or not accessable,')




###### ANGLE ADJUSTMENT STATE PARAMETERS
ROTATING_DELTA_MIN = 100.0
ROTATING_CONST = 10.0
ROTATING_ANGULAR_SPEED = 2.0
NORMALIZED_DELTA_MAX = 0.4
NORMALIZED_DELTA_MIN = 0.4

DEFAULT_MAX_ANGLE_STEP = 15.0
FULL_APP_TIMEOUT = 45.0

###### ATTACHING STATE PARAMETERS
# ATACHING_DISTANCE_MIN = 250
ATACHING_DISTANCE_MIN = 900
# ATACHING_DISTANCE_MAX = 200
ATACHING_DISTANCE_MAX = 370
CART_MAX_DISTANCE = 80.0

###### LINEAR MOVING STATE PARAMETER
MIN_LINEAR_MOVING_VELOCITY = 0.005
MAX_LINEAR_MOVING_VELOCITY = 0.05

###### VERIFICATION STATE PARAMETER
VERIFICATION_DISTANCE = 3.5
POSITION_VERIFICATION_MAX_INDEX = 5.0


###### CART GRIPPER PARAMETERS
GRIPPER_OPEN_PRESSURE_CONST = 1.0
GRIPPER_CLOSE_PRESSURE_CONST = 0.9
GRIPPER_OPEN_POSITION = 1.0
GRIPPER_CLOSE_POSITION = 0.0
GRIPPER_ATTACHED_POSITION = 0.003
GRIPPER_ATTACHED_THRESHOLD = 0.02

###### PRE ATTACH  PARAMETERS
PRE_ATTACH_ANGLE_ROTATION = 180
PRE_ATTACH_RUTATION_SPEED = 10

###### SENSOR PARAMETERS
IR_SENSOR_ID_RIGHT = 2
IR_SENSOR_ID_LEFT = 1
IR_SENSOR_MAX_VALUE_VERIFICATION = 2000
SRF_NOISE_VERIFICATION_DISTANCE = 55.0
SRF_SENSOR_ID_CENTER = 4

###### APPROACH PARAMETERS
CAMERAS_TO_USE = ['cv_top', 'nav_top']
MAX_ANGLE_ERROR_ALLOWED = 1
MAX_Y_ERROR_ALLOWED = 0.02
DEFAULT_FINAL_APPROACH_DISTANCE = 0.5
DEFAULT_FIRST_APPROACH_DISTANCE = 1.0




###### ERRORS

ERROR_CART_NOT_GETTING_CLOSER = (8,'attach failed, cart pushed')
ERROR_GRIPPER_FAILED = (7,'gripper opening failed')
ERROR_ROTATION_MOVEMENT_FAILED = (6,'rotation motion command failed')
ERROR_LINEAR_MOVEMENT_FAILED = (5,
                'gary linear motion command failed')
ERROR_GRIPPER_ATTACHMENT_FAILED = (3,'gripper attachment failed')
ERROR_CART_NOT_ATTACHED = (1,'cart was not attached')
ERROR_TIMEOUT_REACHED = (2,f'loop timeout reached')
ERROR_CART_NOT_ACCESSABLE = (0, 'cart is too far or not accessable,')
ERROR_OBSTACLE_IDENTIFIED = (9,'obstacle detected behind gary')
ERROR_SRF_READING_FAILED = (10,'fail to read SRF value OR got only nan values')

###### Rotating

ROTATING_DISTANCE = 30.0
ROTATING_DISTANCE_AV = 20.0
ROTATING_ANGLE_MIN = 3.5
ROTATING_CONST = 100.0
ROTATING_ANGULAR_SPEED = 30.0
ROTATION_KP = 0.7
MIN_STARTING_ANGLE = 2.0
MAX_ANGLE_CORRECTION_ATTEMPTS = 8.0
MIN_ROTATION_ANGLE_STEP = 1.0
MAX_ANGLE_STEP = 10.0
VERIFICATION_ANGLE = 2.0

###### Attaching
MAX_ATTEMPTS = 8
ATTEMPTS_BEFORE_VIBRATION = 4
ATACHING_DISTANCE_MIN = 4.4
ATACHING_DISTANCE_MAX = 4.8
ATACHING_ANGLE_MAX = 2.0

###### linear movement

MAX_MOVING_VELOCITY = 0.2
VELOCITY_KP = 0.0015
VERIFICATION_VELOCITY = 0.0001

###### Geometric parameters
DISTANCE_BETWEEN_SRF_SENSORS = 23.5
MAX_SRF_VALUE = 150.0
CART_MAX_DISTANCE = 70.0


###### SRF parameters
VERIFICATION_DISTANCE = 1.2
MAX_PUSHING_INDEX = 3.0
FILTER_WEIGHT = 0.1
SRF_SENSOR_ID_RIGHT = '2'
SRF_SENSOR_ID_LEFT = '5'
SRF_SENSOR_ID_MIDDLE = '4'
NOISE_MARGIN = 0.05
POSITION_ERROR_MARGIN = 0.5
# POSITION_ERROR_MARGIN = 0.1


###### pushing cart identifing parameters
PUSHING_IDENTIFIER_DISTANCE = 6.0

###### Obstacle avoidence
OBSTACLE_DISTANCE_PARAMETER = 0.8
MAX_OBSTACLE_INDEX = 5
OBSTACLE_SLEEPING_TIME = 4.0
OBSTACLE_ENABLE_MIN_DISTANCE = 20.0
OBSTACLE_DETECTED_RATIO = 1.5
NON_OBSTACLE_DETECTED_RATIO = 0.6

###### UI
SOUND_VOLUME = 50
REVERSE_BEEPING_ALERT = True ### this version not support beeping
SOUND_NAME = 'beep'
AUDIO_PATH = 'dat:pre_defined_audio'
LOCAL_AUDIO_PATH = 'skills/attach_to_cart/pre_defined_audio'

###### Gripper parameters
GRIPPER_OPEN_PRESSURE_CONST = 0.5
GRIPPER_CLOSE_PRESSURE_CONST = 0.7
GRIPPER_INITIAL_CLOSE_POSITION = 0.4
GRIPPER_INITIAL_CLOSE_PRESSURE= 1.0
GRIPPER_OPEN_POSITION = 1.0
GRIPPER_CLOSE_POSITION = 0.0
GRIPPER_VELOCITY = 1.0
GRIPPER_TIMEOUT = 20.0
MAX_INITIAL_CLOSE_ATTEMPTS = 5

###### extra timeout
TIMEOUT_180 = 20.0

###### DEFAULT_SETUP_ARGS
GRIPPER_ACTUAL_DESIRED_POSITION = 0.01
FULL_APP_TIMEOUT = 70.0
DEBUG = False
DEFUALT_ROTATING_180 = False


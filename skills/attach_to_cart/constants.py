REVERSE = True

SKILL_NAME = 'approach_to_tag'
FAMILY = '36h11'
# TAG_SIZE = 0.12
# SOURCES = ['back']
# TARGET_TAGS = ['1']
TARGET_DISTANCE = 0.7
WAIT_TIME_FOR_DETECTION = 10

MODEL_PARAMS = {
    'families' : 'tag36h11',
    'nthreads' : 4,
    'quad_decimate' : 2.0,
    'quad_sigma': 0.0,
    'decode_sharpening' : 0.25,
    'refine_edges' : 1,
    'tag_size' : 0.0
}

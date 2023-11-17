from enum import Enum

# Please set up the site or coil, choose the robot model, and decide if you want to use force sensing
SITE = "usp_coil"
ROBOT = "dobot" #elfin, dobot, or ur
FORCE_TORQUE_SENSOR = True
DISPLAY_POA = True

TEMP_FILE = 'tmp'

# Define the required offset to align the coil and robot end effector
if SITE == "usp_coil":
    ROBOT_RX_OFFSET = 0
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = 0
elif SITE == "usp_neurosoft":
    ROBOT_RX_OFFSET = -90
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = -30
elif SITE == "aalto":
    ROBOT_RX_OFFSET = 0
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = 157.5
elif SITE == "tubingen":
    ROBOT_RX_OFFSET = 0
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = 157.5
else:
    ROBOT_RX_OFFSET = 0
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = 0

if ROBOT == "dobot":
    # Robot settings
    ROBOT_WORKING_SPACE = 1500  # mm
    SLEEP_ROBOT = 0.005
    # Filtering head motion
    ROBOT_HEAD_VELOCITY_THRESHOLD = 30 #mm/s
    # Tunning motion
    ROBOT_TARGET_TUNING_THRESHOLD_DISTANCE = 50
    ROBOT_TARGET_TUNING_THRESHOLD_ANGLE = 15
    # Arc motion
    ROBOT_ARC_THRESHOLD_DISTANCE = 100 #mm
    ROBOT_ARC_THRESHOLD_DISTANCE_ANGLE = 45 #°
    ROBOT_ARC_BEZIER_CURVE_STEP = 0.4 # step size between 0 and 1. 0.1 generates 10 points; 0.01 generates 100 points along the curve path
    ROBOT_THRESHOLD_TO_UPDATE_TARGET_ARC = 40 #mm
    ROBOT_VERSOR_SCALE_FACTOR = 50
    ROBOT_MIDDLE_ARC_SCALE_FACTOR = 4
    # Check robot tracker registration
    ROBOT_TRANSFORMATION_MATRIX_THRESHOLD = 1 #mm
    # Force and torque sensor
    ROBOT_FORCE_SENSOR_THRESHOLD = 10 #N
    ROBOT_FORCE_SENSOR_SCALE_THRESHOLD = 100 #% of init force
elif ROBOT == "elfin":
    # Robot settings
    # Robot Working Space is defined as 800mm in Elfin 5 manual. For safety, the value is reduced by 5%.
    # For debug, feel free to use 1000 mm
    ROBOT_WORKING_SPACE = 1000 #mm
    SLEEP_ROBOT = 0.0
    # Filtering head motion
    ROBOT_HEAD_VELOCITY_THRESHOLD = 20  # mm/s
    # Tunning motion
    ROBOT_TARGET_TUNING_THRESHOLD_DISTANCE = 30
    ROBOT_TARGET_TUNING_THRESHOLD_ANGLE = 5
    # Arc motion
    ROBOT_ARC_THRESHOLD_DISTANCE = 80  # mm
    ROBOT_ARC_THRESHOLD_DISTANCE_ANGLE = 30  # °
    ROBOT_MIDDLE_ARC_SCALE_FACTOR = 1.5
    ROBOT_THRESHOLD_TO_UPDATE_TARGET_ARC = 40  # mm
    ROBOT_VERSOR_SCALE_FACTOR = 50
    # Check robot tracker registration
    ROBOT_TRANSFORMATION_MATRIX_THRESHOLD = 1  # mm
    # Force and torque sensor
    ROBOT_FORCE_SENSOR_THRESHOLD = 10  # N
    ROBOT_FORCE_SENSOR_SCALE_THRESHOLD = 30  # % of init force

# Please, do not change the following enum.

class MotionType(Enum):
    NORMAL = 0
    LINEAR_OUT = 1
    ARC = 2
    FORCE_LINEAR_OUT = 3
    TUNING = 4

# Publisher messages from invesalius
PUB_MESSAGES = ['Connect to robot',
                'Robot navigation mode',
                'Update robot target',
                'Reset robot process',
                'Update tracker coordinates',
                'Update tracker fiducials matrix',
                'Collect coordinates for the robot transformation matrix',
                'Reset coordinates collection for the robot transformation matrix',
                'Robot transformation matrix estimation',
                'Load robot transformation matrix',
                'Reset robot',
                'Coil at target',
                'Distance to the target',
                ]

FUNCTION_CONNECT_TO_ROBOT = 0
FUNCTION_ROBOT_NAVIGATION_MODE = 1
FUNCTION_UPDATE_ROBOT_TARGET = 2
FUNCTION_RESET_ROBOT_PROCESS = 3
FUNCTION_UPDATE_TRACKER_COORDINATES = 4
FUNCTION_UPDATE_TRACKER_FIDUCIALS = 5
FUNCTION_COLLECT_COORDINATES_TO_ROBOT_MATRIX = 6
FUNCTION_RESET_ROBOT_MATRIX = 7
FUNCTION_ROBOT_MATRIX_ESTIMATION = 8
FUNCTION_LOAD_ROBOT_MATRIX = 9
FUNCTION_RESET_ROBOT = 10
FUNCTION_COIL_AT_TARGET = 11
FUNCTION_DISTANCE_TO_TARGET = 12

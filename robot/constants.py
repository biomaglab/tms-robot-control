#Robot
SITE = "usp_coil"
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

SLEEP_ROBOT = 0.01

ROBOT_ElFIN_PORT = 10003

ROBOT_DOBOT_DASHBOARD_PORT = 29999
ROBOT_DOBOT_MOVE_PORT = 30003
ROBOT_DOBOT_FEED_PORT = 30004
ROBOT_DOBOT_TOOL_ID = 0
ROBOT_DOBOT_TIMEOUT_START_MOTION = 10
ROBOT_DOBOT_TIMEOUT_MOTION = 45

ROBOT_MOTIONS = {"normal": 0, "linear out": 1, "arc": 2, 'force linear out': 3, 'tunning': 4}
ROBOT_HEAD_VELOCITY_THRESHOLD = 20 #mm/s
ROBOT_ARC_THRESHOLD_DISTANCE = 100 #mm
ROBOT_ARC_THRESHOLD_DISTANCE_ANGLE = 45 #°
ROBOT_ARC_BEZIER_CURVE_STEP = 0.1 # step size between 0 and 1. 0.1 generates 10 points; 0.01 generates 100 points along the curve path
ROBOT_THRESHOLD_TO_UPDATE_TARGET_ARC = 40 #mm
ROBOT_VERSOR_SCALE_FACTOR = 50
ROBOT_MIDDLE_ARC_SCALE_FACTOR = 2
ROBOT_TRANSFORMATION_MATRIX_THRESHOLD = 1 #mm
ROBOT_FORCE_SENSOR_THRESHOLD = 10 #N
ROBOT_FORCE_SENSOR_SCALE_THRESHOLD = 30 #% of init force
ROBOT_TARGET_TUNING_THRESHOLD_DISTANCE = 50
ROBOT_TARGET_TUNING_THRESHOLD_ANGLE = 15

FORCE_TORQUE_SENSOR = False

#Robot Working Space is defined as 800mm in Elfin manual. For safety, the value is reduced by 5%.
ROBOT_WORKING_SPACE = 1500 #mm
ROBOT_ELFIN_MOVE_STATE = {"free to move": 0,
                    "in motion": 1009,
                    "waiting for execution": 1013,
                    "error": 1025}
ROBOT_DOBOT_MOVE_STATE = {"error": 9}

#Publisher messages
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

#Robot
SITE = "usp_coil"
# Define the required offset to align the coil and robot end effector
if SITE == "usp_coil":
    ROBOT_RX_OFFSET = 0
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = -90
elif SITE == "usp_neurosoft":
    ROBOT_RX_OFFSET = -90
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = -30
else:
    ROBOT_RX_OFFSET = 0
    ROBOT_RY_OFFSET = 0
    ROBOT_RZ_OFFSET = -22.5

SLEEP_ROBOT = 0.01

ROBOT_ElFIN_PORT = 10003

ROBOT_MOTIONS = {"normal": 0, "linear out": 1, "arc": 2, 'force linear out': 3}
ROBOT_HEAD_VELOCITY_THRESHOLD = 20 #mm/s
ROBOT_ARC_THRESHOLD_DISTANCE = 80 #mm
ROBOT_VERSOR_SCALE_FACTOR = 50
ROBOT_TRANSFORMATION_MATRIX_THRESHOLD = 1 #mm
ROBOT_FORCE_SENSOR_THRESHOLD = 10 #N
ROBOT_FORCE_SENSOR_SCALE_THRESHOLD = 30 #% of init force
ROBOT_TARGET_TUNING_THRESHOLD_DISTANCE = 50

FORCE_TORQUE_SENSOR = False

#Robot Working Space is defined as 800mm in Elfin manual. For safety, the value is reduced by 5%.
ROBOT_WORKING_SPACE = 1000 #mm
ROBOT_MOVE_STATE = {"free to move": 0,
                    "in motion": 1009,
                    "waiting for execution": 1013,
                    "error": 1025}

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
                'Add marker to robot control',
                'Remove multiple markers',
                'Remove all markers',
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
FUNCTION_ADD_MARKER = 10
FUNCTION_REMOVE_MULTIPLE_MARKERS = 11
FUNCTION_REMOVE_ALL_MARKERS = 12
FUNCTION_RESET_ROBOT = 13
FUNCTION_COIL_AT_TARGET = 14
FUNCTION_DISTANCE_TO_TARGET = 15

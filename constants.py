#Robot
SLEEP_ROBOT = 0.01

ROBOT_ElFIN_IP = ['143.107.220.251', '169.254.153.251', '127.0.0.1']
ROBOT_ElFIN_PORT = 10003

ROBOT_MOTIONS = {"normal": 0, "linear out": 1, "arc": 2}
ROBOT_HEAD_VELOCITY_THRESHOLD = 10 #mm/s
ROBOT_ARC_THRESHOLD_DISTANCE = 100 #mm
ROBOT_VERSOR_SCALE_FACTOR = 70

#Robot Working Space is defined as 800mm in Elfin manual. For safety, the value is reduced by 5%.
ROBOT_WORKING_SPACE = 760 #mm
ROBOT_MOVE_STATE = {"free to move": 0,
                    "in motion": 1009,
                    "waiting for execution": 1013,
                    "error": 1025}

#Publisher messages
PUB_MESSAGES = ['Connect to robot',
                'Robot navigation mode',
                'Robot target matrix',
                'Reset robot process',
                'Update tracker coordinates',
                'Update tracker fiducials matrix',
                'Collect coordinates for the robot transformation matrix',
                'Reset coordinates collection for the robot transformation matrix',
                'Robot transformation matrix estimation',
                'Load robot transformation matrix']

FUNCTION_CONNECT_TO_ROBOT = 0
FUNCTION_ROBOT_NAVIGATION_MODE = 1
FUNCTION_ROBOT_TARGET_MATRIX = 2
FUNCTION_RESET_ROBOT_PROCESS = 3
FUNCTION_UPDATE_TRACKER_COORDINATES = 4
FUNCTION_UPDATE_TRACKER_FIDUCIALS = 5
FUNCTION_COLLECT_COORDINATES_TO_ROBOT_MATRIX = 6
FUNCTION_RESET_ROBOT_MATRIX = 7
FUNCTION_ROBOT_MATRIX_ESTIMATION = 8
FUNCTION_LOAD_ROBOT_MATRIX = 9


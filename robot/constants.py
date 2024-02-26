DISPLAY_POA = True

TEMP_FILE = 'tmp'

# Site config
#
# rx_offset, ry_offset, rz_offset:
#   The offset to align the coil and robot end effector (TODO: unit?)

SITE_CONFIG = {
  'usp_coil': {
    'rx_offset': 0,
    'ry_offset': 0,
    'rz_offset': 0,
  },
  'usp_neurosoft': {
    'rx_offset': -90,
    'ry_offset': 0,
    'rz_offset': -30,
  },
  'aalto': {
    'rx_offset': 0,
    'ry_offset': 0,
    'rz_offset': 0,
  },
  'tubingen': {
    'rx_offset': 0,
    'ry_offset': 0,
    'rz_offset': 0,
  },
  'default': {
    'rx_offset': 0,
    'ry_offset': 0,
    'rz_offset': 0,
  },
}

# Robot config
#
# General:
#   Working space: TODO (in mm)
#   Sleep: The time between two cycles of the main loop (in seconds).
#   Waiting time: The time to wait after a movement (in seconds).
#
# Head motion:
#   Head velocity threshold: TODO (in mm/s)
#
# Tuning motion:
#   Target tuning threshold distance: TODO (unit?)
#   Target tuning threshold angle: TODO (in degrees)
#
# Arc motion:
#   Arc threshold distance: TODO (unit?)
#   Arc threshold angle: TODO (in degrees)
#   Arc bezier curve step: Step size between 0 and 1; 0.1 generates 10 points, 0.01 generates 100 points along the curve path.
#   Threshold to update target arc: TODO (in mm)
#   Versor scale factor: TODO
#   Middle arc scale factor: TODO
#
# Robot tracker registration:
#   Transformation matrix threshold: TODO (in mm)
#
# Force and torque sensor:
#   Force sensor threshold: TODO (in Newtons)
#   Sensor scale threshold: TODO (as percentage of initial force)

ROBOT_CONFIG = {
  'dobot': {
    # General
    'working_space_radius': 1500,
    'sleep': 0.005,

    # Head motion
    'head_velocity_threshold': 60,

    # Tuning motion
    'distance_threshold_for_tuning': 50,
    'angular_distance_threshold_for_tuning': 15,

    # Arc motion
    'distance_threshold_for_arc_motion': 100,
    'angular_distance_threshold_for_arc_motion': 45,
    'arc_bezier_curve_step': 0.4,
    'threshold_to_update_arc_motion_target': 40,
    'versor_scale_factor': 50,
    'middle_arc_scale_factor': 4,

    # Robot tracker registration
    'transformation_matrix_threshold': 1,

    # Force and torque sensor
    'force_sensor_threshold': 7,
    'force_sensor_scale_threshold': 100,
  },

  "elfin": {
    # General

    # Robot working space is defined as 800 mm in Elfin 5 manual. For safety, the value is
    # reduced by 5%. For debugging, feel free to use 1000 mm.
    'working_space_radius': 1000,
    'sleep': 0.0,

    # Head motion
    'head_velocity_threshold': 20,

    # Tuning motion
    'distance_threshold_for_tuning': 30,
    'angular_distance_threshold_for_tuning': 5,

    # Arc motion
    'distance_threshold_for_arc_motion': 80,
    'angular_distance_threshold_for_arc_motion': 30,
    'arc_bezier_curve_step': None,
    'threshold_to_update_arc_motion_target': 40,
    'versor_scale_factor': 50,
    'middle_arc_scale_factor': 1.5,

    # Robot tracker registration
    'transformation_matrix_threshold': 1,

    # Force and torque sensor
    'force_sensor_threshold': 10,
    'force_sensor_scale_threshold': 30,
  }
}

# Publisher messages from invesalius
PUB_MESSAGES = ['Connect to robot',
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
                'Send objective from neuronavigation to robot',
                ]

FUNCTION_CONNECT_TO_ROBOT = 0
FUNCTION_UPDATE_ROBOT_TARGET = 1
FUNCTION_RESET_ROBOT_PROCESS = 2
FUNCTION_UPDATE_TRACKER_COORDINATES = 3
FUNCTION_UPDATE_TRACKER_FIDUCIALS = 4
FUNCTION_COLLECT_COORDINATES_TO_ROBOT_MATRIX = 5
FUNCTION_RESET_ROBOT_MATRIX = 6
FUNCTION_ROBOT_MATRIX_ESTIMATION = 7
FUNCTION_LOAD_ROBOT_MATRIX = 8
FUNCTION_RESET_ROBOT = 9
FUNCTION_COIL_AT_TARGET = 10
FUNCTION_DISTANCE_TO_TARGET = 11
FUNCTION_SEND_OBJECTIVE_FROM_NEURONAVIGATION_TO_ROBOT = 12

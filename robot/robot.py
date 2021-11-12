#--------------------------------------------------------------------------
# Software:     InVesalius - Software de Reconstrucao 3D de Imagens Medicas
# Copyright:    (C) 2001  Centro de Pesquisas Renato Archer
# Homepage:     http://www.softwarepublico.gov.br
# Contact:      invesalius@cti.gov.br
# License:      GNU - GPL 2 (LICENSE.txt/LICENCA.txt)
#--------------------------------------------------------------------------
#    Este programa e software livre; voce pode redistribui-lo e/ou
#    modifica-lo sob os termos da Licenca Publica Geral GNU, conforme
#    publicada pela Free Software Foundation; de acordo com a versao 2
#    da Licenca.
#
#    Este programa eh distribuido na expectativa de ser util, mas SEM
#    QUALQUER GARANTIA; sem mesmo a garantia implicita de
#    COMERCIALIZACAO ou de ADEQUACAO A QUALQUER PROPOSITO EM
#    PARTICULAR. Consulte a Licenca Publica Geral GNU para obter mais
#    detalhes.
#--------------------------------------------------------------------------
import numpy as np
import queue
from threading import Lock
from time import sleep
import socketio

import constants as const

import elfin as elfin
import coordinates as coordinates
import elfin_processing as elfin_process


def __on_connect():
    global __connected
    print("Connected to {}".format(__remote_host))
    __connected = True

def __on_disconnect():
    print("Disconnected")
    __connected = False

def __on_message_receive(msg):
    __lock.acquire()
    __buffer.append(msg)
    __lock.release()

def get_buffer():
    global __buffer
    __lock.acquire()
    res = __buffer.copy()
    __buffer = []
    __lock.release()
    return res

def connect():
    __sio.connect(__remote_host)
    while not __connected:
        print("Connecting...")
        sleep(1.0)

def send_message(self, topic, data={}):
    __sio.emit('from_robot', {'topic': topic, 'data': data})


def OnRobotConnection(data):
    robot_IP = data["robot_IP"]
    global trck_init_robot
    trck_init_robot = ElfinRobot(robot_IP)

def OnUpdateRobotTransformationMatrix(data):
    m_tracker_to_robot = data["m_tracker_to_robot"]
    tracker_coordinates.SetTrackerToRobotMatrix(m_tracker_to_robot)
    print("Matrix tracker to robot:", m_tracker_to_robot)

def OnUpdateRobotTargetMatrix(data):
    global robot_tracker_flag, m_change_robot_to_head
    robot_tracker_flag = data["robot_tracker_flag"]
    m_change_robot_to_head = np.array(data["m_change_robot_to_head"])

def OnResetProcessTracker(data):
    process_tracker.__init__()

def OnUpdateCoordinates(data):
    if len(data) > 1: #MUTEX required
        coord = data["coord"]
        markers_flag = data["markers_flag"]
        tracker_coordinates.SetCoordinates(np.vstack([coord[0], coord[1], coord[2]]), markers_flag)

def OnUpdateTrackerFiducialsMatrix(data):
    matrix_tracker_fiducials = np.array(data["matrix_tracker_fiducials"])
    process_tracker.SetMatrixTrackerFiducials(matrix_tracker_fiducials)

def ElfinRobot(robot_IP):
    print("Trying to connect Robot via: ", robot_IP)
    trck_init = elfin.Elfin_Server(robot_IP, const.ROBOT_ElFIN_PORT)
    trck_init.Initialize()
    print('Connect to elfin robot tracking device.')

    # return tracker initialization variable and type of connection
    return trck_init

def get_coordinates_from_tracker_devices():
    coord_robot_raw = trck_init_robot.Run()
    coord_robot = np.array(coord_robot_raw)
    coord_robot[3], coord_robot[5] = coord_robot[5], coord_robot[3]
    robot_coordinates.SetRobotCoordinates(coord_robot)

    coord_raw, markers_flag = tracker_coordinates.GetCoordinates()

    return coord_raw, coord_robot_raw, markers_flag

def robot_motion_reset():
    trck_init_robot.StopRobot()
    global arc_motion_flag, arc_motion_step_flag
    arc_motion_flag = False
    arc_motion_step_flag = const.ROBOT_MOTIONS["normal"]

def robot_move_decision(distance_target, new_robot_coordinates, current_robot_coordinates, current_head_filtered):
    """
    There are two types of robot movements.
    We can imagine in two concentric spheres of different sizes. The inside sphere is to compensate for small head movements.
     It was named "normal" moves.
    The outside sphere is for the arc motion. The arc motion is a safety feature for long robot movements.
     Even for a new target or a sudden huge head movement.
    1) normal:
        A linear move from the actual position until the target position.
        This movement just happens when move distance is below a threshold (const.ROBOT_ARC_THRESHOLD_DISTANCE)
    2) arc motion:
        It can be divided into three parts.
            The first one represents the movement from the inner sphere to the outer sphere.
             The robot moves back using a radial move (it use the center of the head as a reference).
            The second step is the actual arc motion (along the outer sphere).
             A middle point, between the actual position and the target, is required.
            The last step is to make a linear move until the target (goes to the inner sphere)

    """
    global arc_motion_flag, arc_motion_step_flag, target_linear_out, target_arc
    #Check if the target is inside the working space
    if process_tracker.estimate_robot_target_length(new_robot_coordinates) < const.ROBOT_WORKING_SPACE:
        #Check the target distance to define the motion mode
        if distance_target < const.ROBOT_ARC_THRESHOLD_DISTANCE and not arc_motion_flag:
            trck_init_robot.SendCoordinates(new_robot_coordinates, const.ROBOT_MOTIONS["normal"])

        elif distance_target >= const.ROBOT_ARC_THRESHOLD_DISTANCE or arc_motion_flag:
            actual_point = current_robot_coordinates
            if not arc_motion_flag:
                head_center_coordinates = process_tracker.estimate_head_center(current_head_filtered).tolist()

                target_linear_out, target_arc = process_tracker.compute_arc_motion(current_robot_coordinates, head_center_coordinates,
                                                                                                  new_robot_coordinates)
                arc_motion_flag = True
                arc_motion_step_flag = const.ROBOT_MOTIONS["linear out"]

            if arc_motion_flag and arc_motion_step_flag == const.ROBOT_MOTIONS["linear out"]:
                coord = target_linear_out
                if np.allclose(np.array(actual_point), np.array(target_linear_out), 0, 1):
                    arc_motion_step_flag = const.ROBOT_MOTIONS["arc"]
                    coord = target_arc

            elif arc_motion_flag and arc_motion_step_flag == const.ROBOT_MOTIONS["arc"]:
                head_center_coordinates = process_tracker.estimate_head_center(current_head_filtered).tolist()

                _, new_target_arc = process_tracker.compute_arc_motion(current_robot_coordinates, head_center_coordinates,
                                                                            new_robot_coordinates)
                if np.allclose(np.array(new_target_arc[3:-1]), np.array(target_arc[3:-1]), 0, 1):
                    None
                else:
                    if process_tracker.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates) >= \
                            const.ROBOT_ARC_THRESHOLD_DISTANCE*0.8:
                        target_arc = new_target_arc

                coord = target_arc

                if np.allclose(np.array(actual_point), np.array(target_arc[3:-1]), 0, 10):
                    arc_motion_flag = False
                    arc_motion_step_flag = const.ROBOT_MOTIONS["normal"]
                    coord = new_robot_coordinates

            trck_init_robot.SendCoordinates(coord, arc_motion_step_flag)
        robot_status = True
    else:
        print("Head is too far from the robot basis")
        robot_status = False

    return robot_status

def robot_control(current_tracker_coordinates_in_robot, current_robot_coordinates, markers_flag):
    global coord_inv_old
    coord_head_tracker_in_robot = current_tracker_coordinates_in_robot[1]
    marker_head_flag = markers_flag[1]
    #coord_obj_tracker_in_robot = current_tracker_coordinates_in_robot[2]
    #marker_obj_flag = markers_flag[2]
    robot_status = False

    if robot_tracker_flag:
        current_head = coord_head_tracker_in_robot
        if current_head is not None and marker_head_flag:
            current_head_filtered = process_tracker.kalman_filter(current_head)
            if process_tracker.compute_head_move_threshold(current_head_filtered):
                new_robot_coordinates = process_tracker.compute_head_move_compensation(current_head_filtered,
                                                                                m_change_robot_to_head)
                robot_status = True
                if coord_inv_old is None:
                   coord_inv_old = new_robot_coordinates

                if np.allclose(np.array(new_robot_coordinates), np.array(current_robot_coordinates), 0, 0.01):
                    #avoid small movements (0.01 mm)
                    pass
                elif not np.allclose(np.array(new_robot_coordinates), np.array(coord_inv_old), 0, 5):
                    #if the head moves (>5mm) before the robot reach the target
                    trck_init_robot.StopRobot()
                    coord_inv_old = new_robot_coordinates
                else:
                    distance_target = process_tracker.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates)
                    robot_status = robot_move_decision(distance_target, new_robot_coordinates, current_robot_coordinates, current_head_filtered)
                    coord_inv_old = new_robot_coordinates
        else:
            print("Head marker is not visible")
            trck_init_robot.StopRobot()

    return robot_status


buffer = []
remote_host = 'http://127.0.0.1:5000'
__buffer = buffer
__remote_host = remote_host
__connected = False
__sio = socketio.Client()

__sio.on('connect', __on_connect)
#__sio.on('disconnect', __on_disconnect)
__sio.on('to_robot', __on_message_receive)

__lock = Lock()

connect()

trk_init = None
process_tracker = elfin_process.TrackerProcessing()

robot_coordinates = coordinates.RobotCoordinates(__sio)
tracker_coordinates = coordinates.TrackerCoordinates()

robot_tracker_flag = False
target_flag = False
m_change_robot_to_head = None
coord_inv_old = None

process_tracker = process_tracker
trck_init_robot = None

arc_motion_flag = False
arc_motion_step_flag = None
target_linear_out = None
target_linear_in = None
target_arc = None
previous_robot_status = False

if __name__ == '__main__':
    while True:
        buf = get_buffer()
        if len(buf) == 0:
            pass
        elif any(item in [d['topic'] for d in buf] for item in const.PUB_MESSAGES):
            topic = [d['topic'] for d in buf]

            for i in range(len(buf)):
                if topic[i] in const.PUB_MESSAGES:
                    get_function = {const.FUNCTION_CONNECT_TO_ROBOT: OnRobotConnection,
                                    const.FUNCTION_ROBOT_TRANSFORMATION_MATRIX: OnUpdateRobotTransformationMatrix,
                                    const.FUNCTION_ROBOT_TARGET_MATRIX: OnUpdateRobotTargetMatrix,
                                    const.FUNCTION_RESET_ROBOT_PROCESS: OnResetProcessTracker,
                                    const.FUNCTION_UPDATE_TRACKER_COORDINATES: OnUpdateCoordinates,
                                    const.FUNCTION_UPDATE_TRACKER_FIDUCIALS: OnUpdateTrackerFiducialsMatrix}
                    get_function[const.PUB_MESSAGES.index(topic[i])](buf[i]["data"])

        if trck_init_robot:
            current_tracker_coordinates, current_robot_coordinates, markers_flag = get_coordinates_from_tracker_devices()
            robot_status = robot_control(current_tracker_coordinates, current_robot_coordinates, markers_flag)

            if previous_robot_status != robot_status:
                __sio.emit('from_robot', {'topic': 'Update robot status', 'data': {'robot_status': robot_status}})
                previous_robot_status = robot_status

        sleep(const.SLEEP_ROBOT)

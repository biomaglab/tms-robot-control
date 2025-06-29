import numpy as np


class RobotApi():
    """
    An API used internally in InVesalius to communicate with the
    outside world.

    When an event of one of several types happens while running InVesalius, e.g.,
    the coil is moved during neuronavigation, this class is used to update the
    information conveyed by the event.

    When created for the first time, takes a connection object obtained
    from outside InVesalius (see the main entrypoint in app.py).

    The owner of the connection object can update the state of InVesalius by implementing
    functions to set callbacks, used then to communicate the new state to InVesalius (see,
    e.g., one of the callback functions below).

    If connection object is not given or it is None, do not do the updates.
    """
    N_VERTICES_IN_POLYGON = 3

    def __init__(self, connection=None, robot_control=None):
        if connection is not None:
            self.assert_valid(connection)
            self.__set_callbacks(connection)

        self.connection = connection
        self.robot_control = robot_control

    def _hasmethod(self, obj, name):
        return hasattr(obj, name) and callable(getattr(obj, name))

    def assert_valid(self, connection):
        assert self._hasmethod(connection, 'update_robot_status')
        assert self._hasmethod(connection, 'robot_connection_status')
        assert self._hasmethod(connection, 'robot_pose_collected')
        assert self._hasmethod(connection, 'set_objective')
        assert self._hasmethod(connection, 'close_robot_dialog')
        assert self._hasmethod(connection, 'update_robot_transformation_matrix')

    # Functions for InVesalius to send updates.
    def update_target_mode(self, enabled):
        if self.connection is not None:
            self.connection.update_target_mode(
                enabled=enabled,
            )

    # Functions for InVesalius to send updates.
    def update_coil_at_target(self, state):
        if self.connection is not None:
            self.connection.update_coil_at_target(
                state=state
            )

    def update_robot_status(self, success):
        if self.connection is not None:
            self.connection.update_robot_status(
                success=success
            )

    def robot_connection_status(self, success):
        if self.connection is not None:
            self.connection.robot_connection_status(
                success=success
            )

    def robot_pose_collected(self, success):
        if self.connection is not None:
            self.connection.robot_pose_collected(
                success=success
            )

    def set_objective(self, objective):
        if self.connection is not None:
            self.connection.set_objective(
                objective=objective
            )

    # Functions for InVesalius to receive updates via callbacks.
    def __set_callbacks(self, connection):
        connection.set_callback__connect_to_robot(self.connect_to_robot)
        connection.set_callback__update_poses(self.update_poses)
        connection.set_callback__coil_at_target(self.on_coil_at_target)
        connection.set_callback__set_target(self.on_set_target)
        connection.set_callback__unset_target(self.on_unset_target)
        connection.set_callback__set_tracker_fiducials(self.on_set_tracker_fiducials)
        connection.set_callback__collect_robot_pose(self.on_create_point)
        connection.set_callback__reset_robot_transformation_matrix(self.on_reset_robot_matrix)
        connection.set_callback__estimate_robot_transformation_matrix(self.on_robot_matrix_estimation)
        connection.set_callback__set_robot_transformation_matrix(self.on_set_robot_transformation_matrix)
        connection.set_callback__update_displacement_to_target(self.on_update_displacement_to_target)
        connection.set_callback__set_objective(self.on_set_objective)

    def connect_to_robot(self, robot_ip):
        if self.connection is not None:
            data = {'robot_IP': robot_ip}
            self.robot_control.on_robot_connection(data)

    def update_poses(self, poses, visibilities):
        if self.connection is not None:
            data = {'poses': poses, 'visibilities': visibilities}
            self.robot_control.on_update_tracker_poses(data)

    def on_coil_at_target(self, state):
        if self.connection is not None:
            data = {'state': state}
            self.robot_control.on_coil_at_target(data)

    def on_set_target(self, target):
        if self.connection is not None:
            data = {'target': target}
            self.robot_control.on_set_target(data)

    def on_unset_target(self):
        if self.connection is not None:
            data = {'target': False}
            self.robot_control.on_unset_target(data)

    def on_set_tracker_fiducials(self, fiducial_left, fiducial_right, fiducial_nasion):
        if self.connection is not None:
            m_fiducial_left = np.array(fiducial_left, dtype=float).reshape((4, 4)).tolist()
            m_fiducial_right = np.array(fiducial_right, dtype=float).reshape((4, 4)).tolist()
            m_fiducial_nasion = np.array(fiducial_nasion, dtype=float).reshape((4, 4)).tolist()
            data = {'tracker_fiducials': [m_fiducial_left, m_fiducial_right, m_fiducial_nasion]}
            self.robot_control.on_set_tracker_fiducials(data)

    def on_create_point(self, data):
        if self.connection is not None:
            self.robot_control.on_create_point(data)

    def on_reset_robot_matrix(self, data):
        if self.connection is not None:
            self.robot_control.on_reset_robot_matrix()

    def on_robot_matrix_estimation(self, data):
        if self.connection is not None:
            self.robot_control.on_robot_matrix_estimation()

    def on_set_robot_transformation_matrix(self, data):
        if self.connection is not None:
            data = {'data': data}
            self.robot_control.on_set_robot_transformation_matrix(data)

    def on_update_displacement_to_target(self, displacement):
        if self.connection is not None:
            data = {'displacement': displacement}
            self.robot_control.on_update_displacement_to_target(data)

    def on_set_objective(self, objective):
        if self.connection is not None:
            data = {'objective': objective}
            self.robot_control.on_set_objective(data)


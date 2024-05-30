from socket import socket, AF_INET, SOCK_STREAM
import time
from threading import Thread, Event

import numpy as np


class StateConnection(Thread):
    # Use a read-only port for reading the robot state.
    PORT=30011

    # If the time between two consecutive state messages is greater than this threshold,
    # print a warning.
    TIME_BETWEEN_STATE_MESSAGES_WARNING_THRESHOLD=0.2

    def __init__(self, ip):
        """
        Class for periodically polling the Universal Robot state.

        Follows the API manual of Universal Robots:

        https://s3-eu-west-1.amazonaws.com/ur-support-site/16496/ClientInterfaces_Primary.pdf
        """
        Thread.__init__(self)

        self.ip = ip

        self.connected = False
        self.socket = None

        self.state = None

        self.daemon = True

        self.stop_event = Event()

        self.worker_thread = None

    def connect_and_start(self):
        if self.connected:
            print("Already connected to the robot")
            return True

        try:
            new_socket = socket(AF_INET, SOCK_STREAM)
            new_socket.connect((self.ip, self.PORT))

            self.socket = new_socket

            self.connected = True

            self.stop_event.clear()

            self.worker_thread = Thread(target=self.run, daemon=True)
            self.worker_thread.start()

        except socket.error as e:
            print("Failed to connect to the robot")
            print(e)

        return self.connected

    def disconnect_and_stop(self):
        """
        Disconnects from the robot and stops the thread.

        :return: True if successful, otherwise False.
        """
        self.stop_event.set()

        if self.worker_thread:
            self.worker_thread.join()

        success = False

        if not self.connected:
            print("Not connected to the robot, therefore cannot disconnect")
            return success

        try:
            self.socket.close()
            self.connected = False
            success = True
        except:
            print("Failed to disconnect from the robot")

        return success

    def is_state_received(self):
        return self.state is not None        

    def get_bytes_from_socket(self, num_of_bytes):
        if not self.connected:
            print("Not connected to the robot")
            return None

        bytes_received = 0
        data = bytearray(num_of_bytes)
        while bytes_received < num_of_bytes:
            try:
                new_bytes = self.socket.recv(num_of_bytes - bytes_received)
                if len(new_bytes) == 0:
                    raise RuntimeError("Socket connection broken")

                data[bytes_received:bytes_received + len(new_bytes)] = new_bytes
                bytes_received += len(new_bytes)
            except socket.error as e:
                print(f"Socket error: {e}")
                self.socket.close()
                raise

            # If the stop event is set, return None to indicate that the operation was interrupted.
            if self.stop_event.is_set():
                return None

        return data

    def get_message_from_socket(self):
        # Get the message length (first 4 bytes)
        message_length_bytes = self.get_bytes_from_socket(4)
        
        if message_length_bytes is None:
            return None, None

        # Interpret the four bytes as the length of the upcoming message
        message_length = int.from_bytes(message_length_bytes, byteorder='big')

        # Get the message type
        message_type = self.get_bytes_from_socket(1)[0]

        if message_type is None:
            return None, None

        # Get the rest of the message
        message = self.get_bytes_from_socket(message_length - 5)

        if message is None:
            return None, None

        return message_type, message

    def run(self):
        previous_state_message_time = None

        while not self.stop_event.is_set():            
            message_type, data = self.get_message_from_socket()

            if message_type is None:
                if not self.stop_event.is_set():
                    print("Failed to get message from the socket")

                continue

            # If message is not a state message, skip it.
            if message_type != STATE_MESSAGE_TYPE:
                continue

            # Print a warning if the time between two consecutive state messages is too long.
            if previous_state_message_time is not None:
                time_between_messages = time.time() - previous_state_message_time
                if time_between_messages > self.TIME_BETWEEN_STATE_MESSAGES_WARNING_THRESHOLD:
                    print("Warning: Time between consecutive state messages is too long: ", time_between_messages)

            # Store the time of the current state message.
            previous_state_message_time = time.time()

            # Check the length of the message and use the appropriate data type.
            if len(data) == STATE_MESSAGE_LENGTH:
                self.state = np.frombuffer(data, dtype=StateMessageType)

            elif len(data) == STATE_MESSAGE_LENGTH_AFTER_CONFIGURATION_CHANGE:
                self.state = np.frombuffer(data, dtype=StateMessageTypeAfterConfigurationChange)

            else:
                print("Error: Unknown message length:", len(data))

    @property
    def X(self):
        return self.state["CartesianInfo"]["X"][0] if self.state is not None else None

    @property
    def Y(self):
        return self.state["CartesianInfo"]["Y"][0] if self.state is not None else None

    @property
    def Z(self):
        return self.state["CartesianInfo"]["Z"][0] if self.state is not None else None

    @property
    def Rx(self):
        return self.state["CartesianInfo"]["Rx"][0] if self.state is not None else None

    @property
    def Ry(self):
        return self.state["CartesianInfo"]["Ry"][0] if self.state is not None else None

    @property
    def Rz(self):
        return self.state["CartesianInfo"]["Rz"][0] if self.state is not None else None

    def is_moving(self):
        return self.state["RobotMode"]["isProgramRunning"][0] if self.state is not None else None

    def is_error_state(self):
        if self.state is None:
            return None

        is_emergency_stopped = self.state["RobotMode"]["isEmergencyStopped"][0]
        is_protective_stopped = self.state["RobotMode"]["isProtectiveStopped"][0]

        return is_emergency_stopped or is_protective_stopped

    def get_pose(self):
        if self.state is None:
            return None

        return [self.X, self.Y, self.Z, self.Rx, self.Ry, self.Rz]

    def __repr__(self):
        if self.state is None:
            return "No state message received yet"

        # Print X, Y, Z, Rx, Ry, and Rz with three decimal places.
        return "X: {:.3f}, Y: {:.3f}, Z: {:.3f}, Rx: {:.3f}, Ry: {:.3f}, Rz: {:.3f}".format(
            self.X, self.Y, self.Z, self.Rx, self.Ry, self.Rz
        )

    # Unused for now; can be used to find out the order of subpackages in the state message.
    #
    # For instance:
    #
    # subpackages = self._get_subpackages(message)
    # print("Subpackage order: ", [subpackage['type'] for subpackage in subpackages])
    #
    def _get_subpackages(self, message):
        subpackages = []
        message_index = 0
        while message_index < len(message):
            subpackage_length_index = message_index
            subpackage_type_index = message_index + 4

            subpackage_length = int.from_bytes(
                message[subpackage_length_index:subpackage_length_index+4],
                byteorder='big'
            )

            subpackage_data_start_index = message_index + 5
            subpackage_data_end_index = message_index + subpackage_length

            subpackage_type = message[subpackage_type_index]
            data = message[subpackage_data_start_index:subpackage_data_end_index + 1]

            subpackages.append({
                'data': data,
                'type': subpackage_type,
                'length': subpackage_length,
            })
            message_index += subpackage_length

        return subpackages


## Message type definitions
#
# The message type is the first byte of the message sent by Universal Robot. These are
# the message types that are properly documented in the API manual; there are other
# message types as well, but they are not documented, and hence are not listed here.
STATE_MESSAGE_TYPE = 16
VERSION_MESSAGE_TYPE = 20

## Message lengths for different kinds of state messages
#
# Depending on the state of the robot, the state message can have different lengths.
# The lengths are defined below.
STATE_MESSAGE_LENGTH = 711
STATE_MESSAGE_LENGTH_AFTER_CONFIGURATION_CHANGE = 1466


# A state message consists of a sequence of subpackages. Each subpackage has a type.
# The types and their corresponding NumPy data types are defined below.

# Subpackage type: 0
RobotModeType = np.dtype([
    ('packageSize', '>u4'),  # >u4 means unsigned 4-byte integer in big-endian byte order
    ('packageType', '>u1'),  # >u1 means unsigned 1-byte integer in big-endian byte order
    ('timestamp', '>u8'),    # >u8 means unsigned 8-byte integer in big-endian byte order
    ('isRealRobotConnected', np.bool_),
    ('isRealRobotEnabled', np.bool_),
    ('isRobotPowerOn', np.bool_),
    ('isEmergencyStopped', np.bool_),
    ('isProtectiveStopped', np.bool_),
    ('isProgramRunning', np.bool_),
    ('isProgramPaused', np.bool_),
    ('robotMode', '>u1'),
    ('controlMode', '>u1'),
    ('targetSpeedFraction', '>f8'),  # >f8 means 8-byte float in big-endian byte order
    ('speedScaling', '>f8'),
    ('targetSpeedFractionLimit', '>f8'),
    ('reserved', '>u1')
])

# Subpackage type: 1
JointType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('joints', np.dtype([
        ('q_actual', '>f8'),
        ('q_target', '>f8'),
        ('qd_actual', '>f8'),
        ('I_actual', '>f4'),  # >f4 means 4-byte float in big-endian byte order
        ('V_actual', '>f4'),
        ('T_motor', '>f4'),
        ('T_micro', '>f4'),
        ('jointMode', '>u1')
    ]), (6,))
])

# Subpackage type: 2
ToolDataType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('analogInputRange0', '>u1'),
    ('analogInputRange1', '>u1'),
    ('analogInput0', '>f8'),
    ('analogInput1', '>f8'),
    ('toolVoltage48V', '>f4'),
    ('toolOutputVoltage', '>u1'),
    ('toolCurrent', '>f4'),
    ('toolTemperature', '>f4'),
    ('toolMode', '>u1')
])

# Subpackage type: 3
MasterboardDataType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('digitalInputBits', '>u4'),
    ('digitalOutputBits', '>u4'),
    ('analogInputRange0', '>u1'),
    ('analogInputRange1', '>u1'),
    ('analogInput0', '>f8'),
    ('analogInput1', '>f8'),
    ('analogOutputDomain0', '>u1'),
    ('analogOutputDomain1', '>u1'),
    ('analogOutput0', '>f8'),
    ('analogOutput1', '>f8'),
    ('masterBoardTemperature', '>f4'),
    ('robotVoltage48V', '>f4'),
    ('robotCurrent', '>f4'),
    ('masterIOCurrent', '>f4'),
    ('safetyMode', '>u1'),
    ('InReducedMode', '>u1'),
    ('euromap67InterfaceInstalled', '>u1'),
    ('notInUse', '>u4'),
    ('operationalModeSelectorInput', '>u1'),
    ('threePositionEnablingDeviceInput', '>u1'),
    ('notInUse_', '>u1'),
])

# Subpackage type: 4
CartesianInfoType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('X', '>f8'),
    ('Y', '>f8'),
    ('Z', '>f8'),
    ('Rx', '>f8'),
    ('Ry', '>f8'),
    ('Rz', '>f8'),
    ('TCPOffsetX', '>f8'),
    ('TCPOffsetY', '>f8'),
    ('TCPOffsetZ', '>f8'),
    ('TCPOffsetRx', '>f8'),
    ('TCPOffsetRy', '>f8'),
    ('TCPOffsetRz', '>f8'),
])

# Subpackage type: 5
KinematicsInfoType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('joints', np.dtype([
        ('checksum', '>u4'),
        ('DHtheta', '>f8'),
        ('DHa', '>f8'),
        ('Dhd', '>f8'),
        ('Dhalpha', '>f8')
    ]), (6,)),
    ('calibration_status', '>u4')
])

# Subpackage type: 6
ConfigurationDataType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('jointMinLimit', ('>f8', (6,))),
    ('jointMaxLimit', ('>f8', (6,))),
    ('jointMaxSpeed', ('>f8', (6,))),
    ('jointMaxAcceleration', ('>f8', (6,))),
    ('vJointDefault', '>f8'),
    ('aJointDefault', '>f8'),
    ('vToolDefault', '>f8'),
    ('aToolDefault', '>f8'),
    ('eqRadius', '>f8'),
    ('DHa', ('>f8', (6,))),
    ('Dhd', ('>f8', (6,))),
    ('DHalpha', ('>f8', (6,))),
    ('DHtheta', ('>f8', (6,))),
    ('masterboardVersion', '>i4'),  # >i4 means 4-byte integer in big-endian byte order
    ('controllerBoxType', '>i4'),
    ('robotType', '>i4'),
    ('robotSubType', '>i4')
])

# Subpackage type: 7
ForceModeDataType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('Fx', '>f8'),
    ('Fy', '>f8'),
    ('Fz', '>f8'),
    ('Frx', '>f8'),
    ('Fry', '>f8'),
    ('Frz', '>f8'),
    ('robotDexterity', '>f8')
])

# Subpackage type: 8
AdditionalInfoType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('tpButtonState', '>u1'),
    ('freedriveButtonEnabled', np.bool_),
    ('IOEnabledFreedrive', np.bool_),
    ('reserved', '>u1')
])

# Subpackage type: 9
CalibrationDataType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('Fx', '>f8'),
    ('Fy', '>f8'),
    ('Fz', '>f8'),
    ('Frx', '>f8'),
    ('Fry', '>f8'),
    ('Frz', '>f8'),
])

# Subpackage type: 10
SafetyDataType = np.dtype((np.ubyte, (43,)))

# Subpackage type: 11
ToolCommunicationInfoType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('toolCommunicationIsEnabled', np.bool_),
    ('baudRate', '>i4'),
    ('parity', '>i4'),
    ('stopBits', '>i4'),
    ('RxIdleChars', '>f4'),
    ('TxIdleChars', '>f4')
])

# Subpackage type: 12
ToolModeInfoType = np.dtype([
    ('packageSize', '>u4'),
    ('packageType', '>u1'),
    ('output_mode', '>u1'),
    ('digital_output_mode_output_0', '>u1'),
    ('digital_output_mode_output_1', '>u1')
])

# Subpackage type: 14
#
# XXX: This is not documented in the API manual, but it is present in the
#   state message. The length of this subpackage is 85 bytes.
UnknownType = np.dtype((np.ubyte, (85,)))


SubpackageTypes = {
    0: ("RobotMode", RobotModeType),
    1: ("JointData", JointType),
    2: ("ToolData", ToolDataType),
    3: ("MasterboardData", MasterboardDataType),
    4: ("CartesianInfo", CartesianInfoType),
    5: ("KinematicsInfo", KinematicsInfoType),
    6: ("ConfigurationData", ConfigurationDataType),
    7: ("ForceModeData", ForceModeDataType),
    8: ("AdditionalInfo", AdditionalInfoType),
    9: ("CalibrationData", CalibrationDataType),
    10: ("SafetyData", SafetyDataType),
    11: ("ToolCommunicationInfo", ToolCommunicationInfoType),
    12: ("ToolModeInfo", ToolModeInfoType),
    14: ("Unknown", UnknownType),
}

# In the normal state message, the subpackages are in the following order:
StateMessageType = np.dtype([
    SubpackageTypes[0],
    SubpackageTypes[1],
    SubpackageTypes[4],
    SubpackageTypes[9],
    SubpackageTypes[3],
    SubpackageTypes[2],
    SubpackageTypes[7],
    SubpackageTypes[8],
    SubpackageTypes[10],
    SubpackageTypes[11],
    SubpackageTypes[12],
])

# The subpackage sequence differs in the state message received after a configuration change;
# hence, we define a separate data type for that:
StateMessageTypeAfterConfigurationChange = np.dtype([
    SubpackageTypes[0],
    SubpackageTypes[1],
    SubpackageTypes[4],
    SubpackageTypes[5],
    SubpackageTypes[9],
    SubpackageTypes[3],
    SubpackageTypes[2],
    SubpackageTypes[6],
    SubpackageTypes[7],
    SubpackageTypes[8],
    SubpackageTypes[10],
    SubpackageTypes[11],
    SubpackageTypes[12],
    SubpackageTypes[14],
])

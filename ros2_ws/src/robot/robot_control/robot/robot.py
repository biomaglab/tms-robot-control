

from threading import Event

from rclpy.executors import MultiThreadedExecutor

from neuronavigation_interfaces.msg import EulerAngles, PoseUsingEulerAngles, OptitrackPoses, ElectricField

import rclpy

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy, QoSProfile

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

#from robot.control.robot import RobotControl

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.rc = None

        self.logger = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()

        # Create publishers, subscribers, and services
        qos_persist_latest = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        callback_group = ReentrantCallbackGroup()

        self._focus_publisher_subscription = self.create_subscription(PoseUsingEulerAngles, "neuronavigation/focus",
                                                                    self.update_focus, qos_persist_latest, callback_group=callback_group)
        self.event_feedback = {}

    def update_focus(self, msg):
        print("msg: ", msg)
        print("position: ", msg.position)
        print("orientation: ", msg.orientation)
        print("orientation alpha: ", msg.orientation.alpha)
        data = [msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.alpha, msg.orientation.beta, msg.orientation.gamma]
        #RobotControl.OnUpdateCoordinates(data)


def main(args=None):
    rclpy.init(args=args)

    # Allow several actions to be executed concurrently.
    #
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(RobotControlNode(), executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

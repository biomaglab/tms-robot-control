import time

from robot.robots.universal_robot.universal_robot import UniversalRobot

ip = "192.168.5.5"

robot = UniversalRobot(
    ip=ip,
)

for i in range(2):
    success = robot.connect()
    print("Connected: ", success)

    robot.initialize()

    # Get the current pose of the robot.
    target = robot.get_pose()

    print("Current position: X = {:.2f}, Y = {:.2f}, Z = {:.2f}, Rx = {:.2f}, Ry = {:.2f}, Rz = {:.2f}".format(
        target[0], target[1], target[2], target[3], target[4], target[5],
    ))

    # Move the robot to a new position.
    target[0] -= 0.02

    # Send the movement command to the robot.
    response = robot.move_linear(target, 0.1)

    # Wait until the robot has started moving.
    moving = False
    print("")
    print("Waiting for the robot to start moving...")
    while not moving:
        moving = robot.is_moving()
        time.sleep(0.1)

    print("Robot is moving")

    # Wait until the robot has stopped moving.
    print("")
    print("Waiting for the robot to stop moving...")
    while moving:
        moving = robot.is_moving()
        time.sleep(0.1)

    print("Robot has stopped moving")

    # Disconnect from the robot.
    connected = robot.is_connected()
    print("Robot connection status before disconnect: ", connected)

    success = robot.disconnect()
    print("")
    if success:
        print("Disconnected successfully")
    else:
        print("Failed to disconnect")
    print("")

    connected = robot.is_connected()
    print("Robot connection status after disconnect: ", connected)

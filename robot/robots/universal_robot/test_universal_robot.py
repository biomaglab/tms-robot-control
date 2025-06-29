import os
import sys
import time

# All the imports in the module files (e.g., universal_robot.py) assume
# that they are being executed with the repository root as the working directory.
# However, this test script is located in a subdirectory.
# To ensure that all imports work correctly, we need to add the root directory to the Python path.
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
sys.path.insert(0, root_dir)


from robot.robots.universal_robot.universal_robot import UniversalRobot  # noqa: E402

ip = "169.254.128.103"

robot = UniversalRobot(
    ip=ip,
)

# Test the robot movement in both directions.
for movement_direction in [-1, 1]:
    # Connect to the robot.
    success = robot.connect()
    print("Connected: ", success)

    robot.initialize()

    # Get the current pose of the robot.
    current_pose = robot.get_pose()

    print("")
    print(
        "Pose before movement: X = {:.3f}, Y = {:.3f}, Z = {:.3f}, Rx = {:.2f}, Ry = {:.2f}, Rz = {:.2f}".format(
            current_pose[0],
            current_pose[1],
            current_pose[2],
            current_pose[3],
            current_pose[4],
            current_pose[5],
        )
    )

    ## Test linear movement.
    print("")
    print("Press enter to test linear movement")
    input()

    # Send the movement command to the robot.
    target = current_pose[:]
    target[2] -= 3 * movement_direction

    speed_ratio = 0.01

    response = robot.move_linear(
        target=target,
        speed_ratio=speed_ratio,
    )

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

    current_pose = robot.get_pose()

    print("")
    print(
        "Pose after movement: X = {:.3f}, Y = {:.3f}, Z = {:.3f}, Rx = {:.2f}, Ry = {:.2f}, Rz = {:.2f}".format(
            current_pose[0],
            current_pose[1],
            current_pose[2],
            current_pose[3],
            current_pose[4],
            current_pose[5],
        )
    )

    ## Test circular movement.
    print("")
    print("Press enter to test circular movement")
    input()

    # Send the movement command to the robot.
    current_pose = robot.get_pose()

    waypoint = current_pose[:]
    waypoint[0] += 2 * movement_direction
    waypoint[1] += 2 * movement_direction
    waypoint[2] += 2 * movement_direction

    target = waypoint[:]
    target[0] += 2 * movement_direction
    target[1] += 2 * movement_direction
    target[2] += 2 * movement_direction

    speed_ratio = 0.01

    response = robot.move_circular(
        start_position=current_pose,
        waypoint=waypoint,
        target=target,
        speed_ratio=speed_ratio,
    )

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

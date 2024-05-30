import time

from robot.robots.universal_robot.universal_robot import UniversalRobot

ip = "192.168.5.5"

robot = UniversalRobot(
    ip=ip,
)

success = robot.connect()
print("Connected: ", success)

robot.initialize()

# Get the current pose of the robot.

target = robot.get_pose()

print("Current position: ", target)

# Move the robot to a new position.
target[0] -= 0.05

response = robot.move_linear(target, 0.1)

# Check if the robot is moving.
# moving = universal.is_moving()

# print("Is moving: ", moving)
print("Is moving: ")

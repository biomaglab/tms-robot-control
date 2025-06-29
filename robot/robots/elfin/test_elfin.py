import os
import sys

# All the imports in the module files (e.g., elfin.py) assume
# that they are being executed with the repository root as the working directory.
# However, this test script is located in a subdirectory.
# To ensure that all imports work correctly, we need to add the root directory to the Python path.
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
sys.path.insert(0, root_dir)


from robot.robots.elfin.elfin import Elfin

ip = "192.168.200.251"

# Initialize the robot object.
elfin = Elfin(ip=ip, use_new_api=True)

# Connect to the robot.
elfin.connect()

# Initialize the robot.
elfin.initialize()

# Get the current pose of the robot.
success, pose = elfin.get_pose()

print("Current pose: ", pose)

# Copy the current pose to a new variable.
target = pose[:]

# Move the robot to a new pose (1 cm upwards).
target[2] += 10  # in mm

speed = 0.01  # as a proportion of the maximum speed
elfin.move_linear(target=target, speed=speed)

# Check if the robot is moving.
moving = elfin.is_moving()

print("Is moving: ", moving)

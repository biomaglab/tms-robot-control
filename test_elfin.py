from robot.robots.elfin.elfin import Elfin

ip = "192.168.200.251"
config = {
    'robot_speed': 0.05,
}

elfin = Elfin(
    ip=ip,
    config=config,
    use_new_api=False
)

elfin.connect()

elfin.initialize()

# Get the current pose of the robot.
target = elfin.get_pose()

print("Current position: ", target)

# Move the robot to a new position.
target[2] += 10  # in mm

elfin.move_linear(target)

# Check if the robot is moving.
moving = elfin.is_moving()

print("Is moving: ", moving)

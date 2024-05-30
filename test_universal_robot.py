from robot.robots.universal.universal import Universal

ip = "192.168.5.5"
vel = 0.2
acc = 0.2
time = 0
radius = 0
mode = 0
# config = {
#     'vel': 0.2,
#     'acc': 0.2
# }

universal = Universal(
    ip=ip,
    # config=config,
    # use_new_api=False
)

universal.connect()

# universal.initialize()

# Get the current pose of the robot.
# target = universal.get_pose()
target = [25, -209, 588, 0, -2, -1]

print("Current position: ", target)

# Move the robot to a new position.
target[2] += 10  # in mm


response = universal.move_linear(target, vel, acc, time, radius)

# Check if the robot is moving.
# moving = universal.is_moving()

# print("Is moving: ", moving)
print("Is moving: ")

import time

from universal_robot_state import UniversalRobotState

robot_state = UniversalRobotState("192.168.5.5")

robot_state.connect()
robot_state.start()

time.sleep(1)

while True:
    print("")
    print(robot_state)
    print("X-coordinate: " + str(robot_state.X))

    time.sleep(0.2)

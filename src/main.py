from connection import *


if __name__ == '__main__':
    # thread = Debug(['SYS_STATUS'])
    # thread.start()
    drone = DroneBase(socket='172.22.80.1:14550')
    while 1:
        drone.wait_healthy()
        drone.arm()
        drone.guided()
        drone.takeoff(20)
        drone.wait_altitude(19.5)
        drone.land()
        drone.wait_altitude(0.5)
        drone.disarm()
        drone.stabilize()

from connection import *
import time

GSC_HOST = '172.22.84.123'
GSC_PORT = 14550




if __name__ == '__main__':
    thread = Debug(['SYS_STATUS'])
    thread.start()

    wait_healthy()
    arm()
    guided()
    takeoff(20)
    wait_altitude(19.5)
    land()
    wait_altitude(0.5)
    disarm()
    stabilize()

from connection import *
import time

GSC_HOST = '172.22.84.123'
GSC_PORT = 14550




if __name__ == '__main__':
    thread = Debug(['SYS_STATUS'])
    thread.start()
    # unhealthy
    # - onboard_control_sensors_present = 1467087919 / 01010111011100011111110000101111
    # - onboard_control_sensors_enabled = 1398905903 / 01010011011000011001110000101111
    # - onboard_control_sensors_health =  1467063343 / 01010111011100011001110000101111

    # healthy
    # - onboard_control_sensors_present = 1467087919 / 01010111011100011111110000101111
    # - onboard_control_sensors_enabled = 1398930479 / 01010011011000011111110000101111
    # - onboard_control_sensors_health =  1467087919 / 01010111011100011111110000101111

    wait_healthy()
    arm()
    guided()
    takeoff(20)
    wait_altitude(19.5)
    land()
    wait_altitude(0.5)
    disarm()
    stabilize()

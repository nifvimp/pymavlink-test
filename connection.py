import threading
import numpy as np
import time
from pymavlink import mavutil
from constants import GSC_HOST as host
from constants import GSC_PORT as port

connection_string = 'udpin:%s:%s' % (host, port)
connection = mavutil.mavlink_connection(connection_string)
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (connection.target_system, connection.target_component))

class Debug(threading.Thread):
    def __init__(self, param_names):
        threading.Thread.__init__(self)
        self.param_names = param_names

    def run(self):
        while (True):
            for param_name in self.param_names:
                print(connection.recv_match(type=param_name, blocking=True))

    # healthy
    # - onboard_control_sensors_present = 1467087919 / 01010111011100011111110000101111
    # - onboard_control_sensors_enabled = 1398905903 / 01010011011000011001110000101111
    # - onboard_control_sensors_health =  1467063343 / 01010111011100011001110000101111
    # flying
    # - onboard_control_sensors_present = 1467087919 / 01010111011100011111110000101111
    # - onboard_control_sensors_enabled = 1398930479 / 01010011011000011111110000101111
    # - onboard_control_sensors_health =  1467087919 / 01010111011100011111110000101111
def wait_healthy():
    sys_status = connection.recv_match(type='SYS_STATUS', blocking=True)
    while 1467063343 > sys_status.onboard_control_sensors_health:
        sys_status = connection.recv_match(type='SYS_STATUS', blocking=True)
        print("present: {present}, health: {health}".format(
            present=sys_status.onboard_control_sensors_present,
            health=sys_status.onboard_control_sensors_health))
    time.sleep(1)


def wait_altitude(altitude):
    curr_altitude = np.abs(connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
    if (altitude < curr_altitude):
        while(curr_altitude > altitude):
            print("%s < %s" % (altitude, curr_altitude))
            curr_altitude = np.abs(connection.recv_match(type='LOCAL_POSITION_NED', blocking=True))
    elif (altitude > curr_altitude):
        while(curr_altitude < altitude):
            print("%s > %s" % (altitude, curr_altitude))
            curr_altitude = np.abs(connection.recv_match(type='LOCAL_POSITION_NED', blocking=True))
    time.sleep(1)


def guided():
     connection.set_mode(connection.mode_mapping()['GUIDED'])


def stabilize():
    connection.mav.set_mode_send(
         connection.target_system,
         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
         connection.mode_mapping()['STABILIZE'])


def arm():
    connection.arducopter_arm()
    connection.motors_armed_wait()


def disarm():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    connection.motors_disarmed_wait()


def takeoff(altitude):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude)


def land():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)

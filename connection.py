import threading
import numpy as np
import time
from pymavlink import mavutil
from pymavlink.dialects.v20.all import enums
from constants import GSC_HOST as host
from constants import GSC_PORT as port

connection_string = 'udpin:%s:%s' % (host, port)
connection = mavutil.mavlink_connection(connection_string)
heartbeat = connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (connection.target_system, connection.target_component))
print("Available flight modes:\n", connection.mode_mapping())

class Debug(threading.Thread):
    def __init__(self, param_names):
        threading.Thread.__init__(self)
        self.param_names = param_names

    def run(self):
        while (True):
            for param_name in self.param_names:
                print(connection.recv_match(type=param_name, blocking=True))

def cmd_ack():
    ack = connection.recv_match(type='COMMAND_ACK', blocking=True)
    result = ack.result
    cmd_name = enums['MAV_CMD'][ack.command].name
    match result:
        case 0: print("%s: ACCEPTED." % cmd_name) ; return True
        case 1: print("%s: TEMPORARILY REJECTED." % cmd_name)
        case 2: print("%s: DENIED." % cmd_name)
        case 3: print("%s: UNSUPPORTED / UNKNOWN." % cmd_name)
        case 4: print("%s: FAILED." % cmd_name)
        case 5: print("%s: CANCELED." % cmd_name)
        case 6: print("%s: IN-PROGRESS." % cmd_name)
        case 7: print("%s: CMD NOT LONG." % cmd_name)
        case 8: print("%s: CMD NOT INT." % cmd_name)
        case 9: print("%s: UNSUPPORTED MAV FRAME." % cmd_name)
        case _: print("?????????????????????????")
    time.sleep(1)
    return False

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
        # print("present: {present}, health: {health}".format(
        #     present=sys_status.onboard_control_sensors_present,
        #     health=sys_status.onboard_control_sensors_health))
    time.sleep(1)

def wait_altitude(altitude):
    curr_altitude = np.abs(connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
    if (altitude < curr_altitude):
        while(curr_altitude > altitude):
            print("%s < %s" % (altitude, curr_altitude))
            curr_altitude = np.abs(connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
    elif (altitude > curr_altitude):
        while(curr_altitude < altitude):
            print("%s > %s" % (altitude, curr_altitude))
            curr_altitude = np.abs(connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)


def guided():
    accepted = False
    while not accepted:
        connection.set_mode(connection.mode_mapping()['GUIDED'])
        accepted = cmd_ack()


def stabilize():
    accepted = False
    while not accepted:
        connection.set_mode(connection.mode_mapping()['STABILIZE'])
        accepted = cmd_ack()


def arm():
    accepted = False
    while not accepted:
        connection.arducopter_arm()
        accepted = cmd_ack()
    connection.motors_armed_wait()


def disarm():
    accepted = False
    while not accepted:
        connection.arducopter_disarm()
        accepted = cmd_ack()
    connection.motors_disarmed_wait()

def takeoff(altitude):
    accepted = False
    while not accepted:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude)
        accepted = cmd_ack()

def land():
    accepted = False
    while not accepted:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0)
        accepted = cmd_ack()
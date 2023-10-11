from pymavlink import mavutil
from constants import GSC_HOST as host
from constants import GSC_PORT as port

connection_string = 'udpin:%s:%s' % (host, port)
connection = mavutil.mavlink_connection(connection_string)
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (connection.target_system, connection.target_component))

def guided():
    connection.mav.set_mode_send(
         connection.target_system,
         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
         connection.mode_mapping()['GUIDED'])


def stabilize():
    connection.mav.set_mode_send(
         connection.target_system,
         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
         connection.mod_mapping()['STALIZE'])


def arm():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)


def disarm():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)


def takeoff(altitude):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)

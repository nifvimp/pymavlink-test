from pymavlink import mavutil
from constants import GSC_HOST as host
from constants import GSC_PORT as port

connection = mavutil.mavlink_connection('udpin:%u:%u' % (host, port))
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (connection.target_system, connection.target_component))

# msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
# print(msg)

def arm():
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1, 0, 0, 0, 0, 0, 0)


def disarm():
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 0, 0, 0, 0, 0, 0, 0)


def takeoff(altitude):
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_TAKEOFF,
                                     0, 0, 0, 0, 0, 0, 0, altitude)

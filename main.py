from pymavlink import mavutil
import time

GSC_HOST = '172.22.84.123'
GSC_PORT = 14550

def takeoff(connection, altitude):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude)


def land(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)

if __name__ == '__main__':
    connection_string = 'udpin:%s:%s' % (GSC_HOST, GSC_PORT)
    connection = mavutil.mavlink_connection(connection_string)
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (connection.target_system, connection.target_component))

    connection.set_mode(mode=connection.mode_mapping()['GUIDED'])
    connection.arducopter_arm()
    land()

    takeoff(connection, 20)
    time.sleep(10)
    land(connection)


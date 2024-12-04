import time

from dronekit import connect, VehicleMode, LocationGlobalRelative

MAP_ORIGIN = -35.3632621, 149.1652374
ARDUPILOT_PORT = 14550


if __name__ == '__main__':
    # Initialize dronekit connection
    connection_string = f'127.0.0.1:{ARDUPILOT_PORT}'
    drone = connect(connection_string, wait_ready=True)

    # Wait for vehicle to initialize
    sys_status = drone._master.recv_match(type='SYS_STATUS', blocking=True)
    while 0 != sys_status.onboard_control_sensors_enabled & ~sys_status.onboard_control_sensors_health:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
        sys_status = drone._master.recv_match(type='SYS_STATUS', blocking=True)

    # Do cool stuff
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    target_alt = 5

    drone.simple_takeoff(target_alt)
    while not drone.location.global_relative_frame.alt >= target_alt * 0.95:
        time.sleep(1)

    drone.mode = VehicleMode("LAND")
    while not drone.location.global_relative_frame.alt <= .5:
        time.sleep(1)

    drone.armed = False
    drone.mode = VehicleMode("STABILIZE")

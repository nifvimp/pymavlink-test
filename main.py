from src.connection import *

GSC_HOST = '172.22.84.123'
GSC_PORT = 14550




if __name__ == '__main__':
    # thread = Debug(['SYS_STATUS'])
    # thread.start()
    connection_string = 'udpin:%s:%s' % (GSC_HOST, GSC_PORT)
    drone = DroneBase(socket=connection_string)
    drone.wait_healthy()
    drone.arm()
    drone.guided()
    print(drone.connection.param_fetch_all())
    drone.takeoff(20)
    drone.wait_altitude(19.5)
    drone.land()
    drone.wait_altitude(0.5)
    drone.disarm()
    drone.stabilize()

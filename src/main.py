from connection import *

GSC_HOST = '127.0.0.1'
GSC_PORT = 5760

if __name__ == '__main__':
    # thread = Debug(['SYS_STATUS'])
    # thread.start()
    connection_string = 'tcp:%s:%s' % (GSC_HOST, GSC_PORT)
    drone = DroneBase(socket=connection_string)
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

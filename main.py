import connection
import time

GSC_HOST = '172.22.84.123'
GSC_PORT = 14550




if __name__ == '__main__':
    connection.arm()
    connection.guided()
    time.sleep(1)
    connection.takeoff(20)
    connection.wait_altitude(19.5)
    time.sleep(1)
    connection.land()
    connection.wait_altitude(0.5)
    time.sleep(1)
    connection.disarm()
    connection.stabilize()

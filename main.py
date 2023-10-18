import connection
import time

GSC_HOST = '172.22.84.123'
GSC_PORT = 14550


if __name__ == '__main__':
    connection.guided()
    connection.arm()
    connection.takeoff(20)
    time.sleep(10)
    connection.land()


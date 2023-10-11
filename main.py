import time
import connection

if __name__ == '__main__':
    connection.guided()
    connection.arm()
    connection.takeoff(20)
    time.sleep(10)
    connection.land()
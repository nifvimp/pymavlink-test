import time
import connection

if __name__ == '__main__':
    connection.guided()
    connection.arm()
    connection.motors_armed_wait()
    connection.takeoff(20)
    time.sleep(10)
    connection.land()
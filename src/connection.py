import os
import threading
import time
import traceback
from queue import Queue
from threading import Thread
from typing import Optional, Union
import atexit

import numpy as np
import pymavlink.dialects.v20.all as mavlink
from loguru import logger as log
from pymavlink import mavutil
from pymavlink.CSVReader import CSVReader
from pymavlink.DFReader import DFReader_binary, DFReader_text
from pymavlink.dialects.v20.all import enums
from pymavlink.mavutil import mavtcp, mavserial, mavlogfile, mavmmaplog, mavtcpin, mavudp, mavmcast, mavchildexec


class DroneBase:
    def __init__(self,
                 socket: str,
                 baudrate: int = 115200,
                 location_tolerance: float = 0.5,
                 dialect: str = None,
                 heartbeat_timeout: float = 10,
                 udp_timeout: float = 1,
                 source_system: int = 255,
                 source_component: int = mavlink.MAV_COMP_ID_ALL
                 ):

        # init params TODO: make params into @properties w/ getters
        # mavlink params
        self.socket: str = socket
        self.dialect: str = dialect
        self.mav_state: int = mavlink.MAV_STATE_UNINIT
        self.source_system: int = source_system
        self.source_component: int = source_component
        self.baudrate: int = baudrate
        self.udp_timeout: float = udp_timeout

        # user defined
        self.location_tolerance: float = location_tolerance
        self.heartbeat_timeout: float = heartbeat_timeout

        # debug
        self._start_time: float = time.time()
        self.last_heartbeat: float = 0

        # fancy
        self.msg_listeners: list = []
        self.msg_handlers: dict = {}
        self.recurring: dict = {}
        self.msg_queue: Queue = Queue()
        self.thread_out: Optional[Thread] = None

        # flags
        self.alive: bool = False
        self.accept_cmd: bool = False

        # connect to drone
        # https://mavlink.io/en/mavgen_python/#setting_up_connection
        os.environ["MAVLINK20"] = "1"  # force mavlink 2.0
        self.connection = self._connect()

        # TODO: setup config // Camera stuff and drone stuff

        # on exit
        def onexit():
            if self.alive:
                self.stop()

        atexit.register(onexit)

        # wait for heartbeat
        heartbeat = self.connection.wait_heartbeat(timeout=heartbeat_timeout)
        print(heartbeat)
        log.info("Heartbeat from system (system %u component %u)" %
                 (self.connection.target_system, self.connection.target_component))

        # default callbacks
        self.register_callback(self.message_handler)
        # self.register_callback(lambda self, msg: log.info(msg))

        # default handler
        self.register_handler('COMMAND_ACK', self.cmd_ack)

        # set stream rates to be faster
        # self.set_stream_rate(4, mavlink.MAV_DATA_STREAM_ALL)
        # self.set_message_rate(1, mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
        # self.set_message_rate(20, mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        # self.set_message_rate(20, mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        # self.set_message_rate(20, mavlink.MAVLINK_MSG_ID_ATTITUDE)

        # init threads
        self.thread_out = Thread(target=self._run, daemon=True)

    def _connect(self) -> (mavtcp | mavtcpin | mavudp | mavmcast | DFReader_binary | CSVReader
                           | DFReader_text | mavchildexec | mavmmaplog | mavlogfile | mavserial):
        return mavutil.mavlink_connection(self.socket,
                                          baud=self.baudrate,
                                          autoreconnect=True,
                                          dialect=self.dialect,
                                          source_component=self.source_component,
                                          source_system=self.source_component,
                                          udp_timeout=self.udp_timeout,
                                          )

    def start(self):
        self.alive = True
        if not self.thread_out.is_alive():
            self.thread_out.start()

    def stop(self):
        self.alive = False
        if self.thread_out is not None:
            self.thread_out.join(timeout=10)
            self.thread_out = None
        self.connection.close()

    def reset(self):
        self.msg_queue = Queue()
        try:
            self.connection.close()
        except:
            pass

        self.connection = self._connect()
        for handler in self.msg_handlers:
            self.connection.message_hooks.append(handler)

    #####################################################################################
    #                            CALLBACKS AND HANDLERS                                 #
    #####################################################################################
    def _run(self):
        while self.alive:
            if not self.msg_queue.empty():
                self.msg_queue.get()()  # TODO: figure out

    def message_handler(self, _, msg):
        func_lst = self.msg_handlers.get(msg.get_type(), [])
        for func in func_lst:
            func(msg)

    def register_callback(self, func):
        self.msg_listeners.append(func)
        self.connection.message_hooks.append(func)

    def unregister_callback(self, func):
        self.connection.message_hooks.remove(func)
        self.msg_listeners.remove(func)

    def register_handler(self, name, func):
        func_lst = self.msg_handlers.get(name, [])
        func_lst.append(func)
        self.msg_handlers[name] = func_lst

    def unregister_handler(self, name, func):
        func_lst = self.msg_handlers[name]
        try:
            func_lst.remove(func)
        finally:
            pass

    # source: https://stackoverflow.com/questions/474528/how-to-repeatedly-execute-a-function-every-x-seconds
    def register_recurring(self, func, interval=1.0):
        # TODO: try to combine all recurring into one thread
        class PeriodicThread(Thread):
            def __init__(self):
                super(PeriodicThread, self).__init__(target=lambda: self._every(func, interval))
                self._cancel_event = threading.Event()

            def cancel(self):
                self._cancel_event.set()

            def _every(self, delay, task):
                next_time = time.time() + delay
                while not self._cancel_event.is_set():
                    time.sleep(max(0, next_time - time.time()))
                    try:
                        task()
                    except Exception:
                        traceback.print_exc()

                    next_time += (time.time() - next_time) // delay * delay + delay

        t = PeriodicThread()
        self.recurring[func] = t
        t.start()

    def unregister_recurring(self, func):
        t = self.recurring.get(func, None)
        if t is not None:
            t.cancel()
            t.join()
            self.recurring.pop(func)

    #####################################################################################
    #                                   DECORATORS                                      #
    #####################################################################################
    def callback(self):
        def decorator(fn):
            self.register_callback(fn)

        return decorator

    def handler(self, name: Union[str, list]):
        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.register_handler(n, fn)
            else:
                self.register_handler(name, fn)

        return decorator

    # WARNING: Uses threads to achieve functionality
    #          could be bad if too many reoccurring
    #          tasks are registered.
    def schedule(self, delay: float):
        def decorator(fn):
            self.register_recurring(fn, interval=delay)

        return decorator

    #####################################################################################
    #                                     UTILITY                                       #
    #####################################################################################
    def set_stream_rate(self, hz, stream=mavlink.MAV_DATA_STREAM_ALL):
        """ Set the stream rate of data from the drone """
        # https://ardupilot.org/dev/docs/mavlink-requesting-data.html
        # NOTE: mavproxy and the GCS will override this
        # run `set streamrate -1` to disable in mavproxy, and look in GCS settings
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,  # target_system
            mavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
            stream,  # stream
            hz,  # rate
            1)  # start/stop

    def send_command_long(self, command, param1=0,
                          param2=0, param3=0,
                          param4=0, param5=0,
                          param6=0, param7=0,
                          wait_ack=False,
                          retries=2):
        """ Send a command to the drone """

        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            command,  # command
            0,  # confirmation
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7)

    def cmd_ack(self, ack):
        result = ack.result
        cmd_name = enums['MAV_CMD'][ack.command].name
        match result:
            case 0:
                log.info("%s: ACCEPTED." % cmd_name)
            case 1:
                log.info("%s: TEMPORARILY REJECTED." % cmd_name)
            case 2:
                log.info("%s: DENIED." % cmd_name)
            case 3:
                log.info("%s: UNSUPPORTED / UNKNOWN." % cmd_name)
            case 4:
                log.info("%s: FAILED." % cmd_name)
            case 5:
                log.info("%s: CANCELED." % cmd_name)
            case 6:
                log.info("%s: IN-PROGRESS." % cmd_name)
            case 7:
                log.info("%s: CMD NOT LONG." % cmd_name)
            case 8:
                log.info("%s: CMD NOT INT." % cmd_name)
            case 9:
                log.info("%s: UNSUPPORTED MAV FRAME." % cmd_name)
            case _:
                log.info("???")
        return result == 0

    def wait_healthy(self):
        sys_status = self.connection.recv_match(type='SYS_STATUS', blocking=True)
        while 0 != sys_status.onboard_control_sensors_enabled & ~sys_status.onboard_control_sensors_health:
            # print(sys_status.onboard_control_sensors_enabled & ~sys_status.onboard_control_sensors_health)
            sys_status = self.connection.recv_match(type='SYS_STATUS', blocking=True)
            # print("present: {present}, health: {health}".format(
            #     present=sys_status.onboard_control_sensors_present,
            #     health=sys_status.onboard_control_sensors_health))
        self.accept_cmd = True

    def wait_altitude(self, altitude: float):
        curr_altitude = np.abs(self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
        if (altitude < curr_altitude):
            while (curr_altitude > altitude):
                # print("%s < %s" % (altitude, curr_altitude))
                curr_altitude = np.abs(self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
        elif (altitude > curr_altitude):
            while (curr_altitude < altitude):
                # print("%s > %s" % (altitude, curr_altitude))
                curr_altitude = np.abs(self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)

    def set_mode(self, mode: str):
        if mode not in self.connection.mode_mapping():
            return
        # accepted = False
        # while not accepted:
        self.connection.set_mode(self.connection.mode_mapping()[mode])
        # accepted = self.cmd_ack()
        # if not accepted:
        #     print("set_mode failed: MODE %s" % mode)
        #     time.sleep(1)

    def stabilize(self):
        self.set_mode('STABILIZE')

    def guided(self):
        self.set_mode('GUIDED')

    def arm(self):
        accepted = False
        # while accepted:
        self.connection.arducopter_arm()
        # accepted = self.cmd_ack()
        self.connection.motors_armed_wait()

    def disarm(self):
        accepted = False
        # while not accepted:
        self.connection.arducopter_disarm()
        # accepted = self.cmd_ack()
        self.connection.motors_disarmed_wait()

    def takeoff(self, altitude: float):
        # accepted = False
        # while not accepted:
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude)
        # accepted = self.cmd_ack()

    def land(self):
        # accepted = False
        # while not accepted:
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0)
        # accepted = self.cmd_ack()

    def move_to_position(self, north, east, down):
        self.connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0)  # yaw, yaw_rate

        # Monitor if we reached the target
        msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        while abs(msg.x - north) > 0.5 or abs(msg.y - east) > 0.5 or abs(msg.z - down) > 0.5:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            # print(f"Current position: ({msg.x}, {msg.y}, {msg.z})")

    def __str__(self) -> str:
        return "Drone ;p;"


class Attribute:
    observers = []

    def __init__(self, name: str, drone: DroneBase):
        setattr(drone, name, self)
        self._drone = drone
        self._name = name

    def update(self, name: Union[str, list[str]]):
        def decorator(fn):
            @self._drone.handler(name)
            def wrapper(msg):
                fn(msg)
                for observer in self.observers:
                    observer(self)

        return decorator

    def subscribe(self):
        def decorator(fn):
            self.observers.append(fn)

        return decorator


class LocalLocation(Attribute):
    def __init__(self, drone: DroneBase):
        super().__init__("local_location", drone)

        @self.update('LOCAL_POSITION_NED')
        def listener(_, msg):
            self._x = msg.x  # +north
            self._y = msg.y  # +east
            self._z = msg.z  # +down
            self._vx = msg.vx
            self._vy = msg.vy
            self._vz = msg.vz


class GlobalLocation(Attribute):
    def __init__(self, drone: DroneBase):
        super().__init__("global_location", drone)

        @self.update('GLOBAL_POSITION_INT')
        def listener(_, msg):
            self._lat = msg.lat
            self._lon = msg.lon
            self._alt = msg.alt
            self._relative_alt = msg.relative_alt

        @self.update('GPS_RAW_INT')
        def listener(_, msg):
            self._rawlat = msg.lat
            self._rawlon = msg.lon
            self._rawalt = msg.alt


"""
SIMSTATE
    {roll: -5.571140718529932e-05, pitch: 7.679801638005301e-05, yaw: -0.0026232670061290264,
     xacc: -7.704182962697814e-07, yacc: -1.2247164704604074e-06, zacc: -9.791999816894531, xgyro: 6.25365792075172e-05,
     ygyro: -3.933918196707964e-05, zgyro: -0.00018548977095633745, lat: -353632621, lng: 1491652374}
AHRS2
    {roll: 0.0017088170861825347, pitch: 0.0016750365030020475, yaw: -0.005486271809786558, altitude: 584.219970703125,
     lat: -353632621, lng: 1491652374}
TERRAIN_REPORT
    {lat: -353632621, lon: 1491652374, spacing: 100, terrain_height: 584.0499877929688,
     current_height: 0.08001708984375, pending: 0, loaded: 504}
EKF_STATUS_REPORT
    {flags: 895, velocity_variance: 0.0, pos_horiz_variance: 0.0, pos_vert_variance: 0.0, compass_variance: 0.0,
     terrain_alt_variance: 0.0, airspeed_variance: 0.0}
LOCAL_POSITION_NED
    {time_boot_ms: 93881, x: 0.0, y: 0.0, z: -0.12999999523162842, vx: -0.0006500626332126558,
     vy: -8.525041630491614e-05, vz: 0.5000156760215759}
VIBRATION
    {time_usec: 93881457, vibration_x: 0.002591438591480255, vibration_y: 0.002407884458079934,
     vibration_z: 0.013609813526272774, clipping_0: 0, clipping_1: 0, clipping_2: 0}
BATTERY_STATUS
    {id: 0, battery_function: 0, type: 0, temperature: 32767,
     voltages: [12600, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535], current_battery: 0,
     current_consumed: 0, energy_consumed: 0, battery_remaining: 100, time_remaining: 0, charge_state: 1,
     voltages_ext: [0, 0, 0, 0], mode: 0, fault_bitmask: 0}
ATTITUDE
    {time_boot_ms: 94133, roll: -5.023180347052403e-05, pitch: 8.340366184711456e-05, yaw: -0.002654439304023981,
     rollspeed: 0.0013135914923623204, pitchspeed: 0.0016029796097427607, yawspeed: 0.0012919099535793066}
GLOBAL_POSITION_INT
    {time_boot_ms: 94133, lat: -353632621, lon: 1491652374, alt: 584060, relative_alt: 10, vx: 0, vy: 0, vz: -10,
     hdg: 35985}
SYS_STATUS
    {onboard_control_sensors_present: 1467087919, onboard_control_sensors_enabled: 1398930479,
     onboard_control_sensors_health: 1467087919, load: 0, voltage_battery: 12600, current_battery: 0,
     battery_remaining: 100, drop_rate_comm: 0, errors_comm: 0, errors_count1: 0, errors_count2: 0, errors_count3: 0,
     errors_count4: 0}
POWER_STATUS
    {Vcc: 5000, Vservo: 0, flags: 0}
MEMINFO
    {brkval: 0, freemem: 65535, freemem32: 131072}
NAV_CONTROLLER_OUTPUT
    {nav_roll: 0.15103793144226074, nav_pitch: 0.15703052282333374, nav_bearing: 0, target_bearing: 0, wp_dist: 0,
     alt_error: -0.04989273473620415, aspd_error: 0.0, xtrack_error: 0.0}
MISSION_CURRENT
    {seq: 0, total: 0, mission_state: 1, mission_mode: 0}
VFR_HUD
    {airspeed: 0.0, groundspeed: 8.063080895226449e-05, heading: 359, throttle: 0, alt: 584.0599975585938,
     climb: 0.10365071892738342}
SERVO_OUTPUT_RAW
    {time_usec: 94133373, port: 0, servo1_raw: 1150, servo2_raw: 1150, servo3_raw: 1150, servo4_raw: 1150,
     servo5_raw: 0, servo6_raw: 0, servo7_raw: 0, servo8_raw: 0, servo9_raw: 0, servo10_raw: 0, servo11_raw: 0,
     servo12_raw: 0, servo13_raw: 0, servo14_raw: 0, servo15_raw: 0, servo16_raw: 0}
RC_CHANNELS
    {time_boot_ms: 94133, chancount: 16, chan1_raw: 1500, chan2_raw: 1500, chan3_raw: 1000, chan4_raw: 1500,
     chan5_raw: 1800, chan6_raw: 1000, chan7_raw: 1000, chan8_raw: 1800, chan9_raw: 0, chan10_raw: 0, chan11_raw: 0,
     chan12_raw: 0, chan13_raw: 0, chan14_raw: 0, chan15_raw: 0, chan16_raw: 0, chan17_raw: 0, chan18_raw: 0, rssi: 255}
RAW_IMU
    {time_usec: 94133373, xacc: 0, yacc: 0, zacc: -876, xgyro: 1, ygyro: 1, zgyro: 1, xmag: 232, ymag: 53, zmag: -528,
     id: 0, temperature: 3021}
CALED_IMU2
    {time_boot_ms: 94133, xacc: 0, yacc: 0, zacc: -869, xgyro: 1, ygyro: 1, zgyro: 1, xmag: 232, ymag: 53, zmag: -528,
     temperature: 3021}
 SCALED_IMU3
    {time_boot_ms: 94133, xacc: 0, yacc: 0, zacc: 0, xgyro: 0, ygyro: 0, zgyro: 0, xmag: 232, ymag: 53, zmag: -528,
     temperature: 0}
SCALED_PRESSURE
    {time_boot_ms: 94133, press_abs: 945.03125, press_diff: 0.0, temperature: 3093, temperature_press_diff: 0}
SCALED_PRESSURE2
    {time_boot_ms: 94133, press_abs: 945.04833984375, press_diff: 0.0, temperature: 3093, temperature_press_diff: 0}
GPS_RAW_INT
    {time_usec: 93990000, fix_type: 6, lat: -353632621, lon: 1491652374, alt: 584120, eph: 121, epv: 200, vel: 0,
     cog: 18854, satellites_visible: 10, alt_ellipsoid: 584120, h_acc: 300, v_acc: 300, vel_acc: 40, hdg_acc: 0, yaw: 0}
SYSTEM_TIME
    {time_unix_usec: 1708481173958486, time_boot_ms: 94133}
AHRS
    {omegaIx: 0.0, omegaIy: 0.0, omegaIz: 0.0, accel_weight: 0.0, renorm_val: 0.0, error_rp: 0.0023464129772037268,
     error_yaw: 0.001737971673719585}
     """

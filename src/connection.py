import os
import threading
import time
import traceback
from queue import Queue
from threading import Thread
from typing import Optional

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
        self.connection.message_hooks.append(self.message_handler)

        # TODO: setup config // Camera stuff and drone stuff

        # on exit
        import atexit
        def onexit(): self.stop()

        atexit.register(onexit)

        @self.callback()
        def print(_, msg):
            print(msg)

        # wait for heartbeat
        heartbeat = self.connection.wait_heartbeat(timeout=heartbeat_timeout)
        print(heartbeat)
        log.info("Heartbeat from system (system %u component %u)" %
                 (self.connection.target_system, self.connection.target_component))

    def message_handler(self, msg):
        func = self.msg_handlers[msg.get_type()]
        if func is not None:
            func(self, msg)

        # init threads
        self.thread_out = Thread(target=self._run, daemon=True)

    def _run(self):
        q = self.msg_queue
        while self.alive:
            if not q.empty():
                q.get()()

    def register_callback(self, func):
        self.msg_listeners.append(func)
        self.connection.message_hooks.append(func)

    def unregister_callback(self, func):
        self.connection.message_hooks.remove(func)
        self.msg_listeners.remove(func)

    def register_handler(self, name, func):
        self.msg_handlers[name] = func

    def unregister_handler(self, name):
        self.msg_handlers.pop(name)

    def register_recurring(self, func, interval=1):
        # source: https://stackoverflow.com/questions/474528/how-to-repeatedly-execute-a-function-every-x-seconds
        class PeriodicThread(Thread):
            def __init__(self):
                super(PeriodicThread, self).__init__(target=lambda: self._every(func, interval))
                self._cancel_event = threading.Event()

            def cancel(self):
                self._cancel_event.set()

            def _every(self, delay, task):
                next_time = time.time() + delay
                while self._cancel_event.is_set():
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
        t = self.recurring[func]
        t.cancel()
        t.join()
        self.recurring.pop(func)

    def callback(self):
        def decorator(fn):
            self.register_callback(fn)

        return decorator

    def handler(self, name):
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
    def schedule(self, delay):
        def decorator(fn):
            self.register_recurring(fn, interval=delay)

        return decorator

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

    def reset(self):
        self.msg_queue = Queue()
        try:
            self.connection.close()
        except:
            pass
        self.connection = self._connect()
        # TODO: re-register all callbacks and such

    def stop(self):
        self.alive = False
        if self.thread_out is not None:
            self.thread_out.join()
            self.thread_out = None
        self.connection.close()

    def __str__(self) -> str:
        return "Drone ;p;"

    def cmd_ack(self):
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        result = ack.result
        cmd_name = enums['MAV_CMD'][ack.command].name
        match result:
            case 0:
                log.info("%s: ACCEPTED." % cmd_name)
                return True
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
        return False

    def wait_healthy(self):
        sys_status = self.connection.recv_match(type='SYS_STATUS', blocking=True)
        while 0 != sys_status.onboard_control_sensors_present ^ sys_status.onboard_control_sensors_health:
            sys_status = self.connection.recv_match(type='SYS_STATUS', blocking=True)
            # print("present: {present}, health: {health}".format(
            #     present=sys_status.onboard_control_sensors_present,
            #     health=sys_status.onboard_control_sensors_health))
        self.accept_cmd = True

    def wait_altitude(self, altitude: float):
        curr_altitude = np.abs(self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
        if (altitude < curr_altitude):
            while (curr_altitude > altitude):
                print("%s < %s" % (altitude, curr_altitude))
                curr_altitude = np.abs(self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)
        elif (altitude > curr_altitude):
            while (curr_altitude < altitude):
                print("%s > %s" % (altitude, curr_altitude))
                curr_altitude = np.abs(self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)

    def set_mode(self, mode: str):
        if mode not in self.connection.mode_mapping():
            return
        accepted = False
        while not accepted:
            self.connection.set_mode(self.connection.mode_mapping()[mode])
            accepted = self.cmd_ack()
            if not accepted:
                print("set_mode failed: MODE %s" % mode)
                time.sleep(1)

    def stabilize(self):
        self.set_mode('STABILIZE')

    def guided(self):
        self.set_mode('GUIDED')

    def arm(self):
        accepted = False
        while not accepted:
            self.connection.arducopter_arm()
            accepted = self.cmd_ack()
        self.connection.motors_armed_wait()

    def disarm(self):
        accepted = False
        while not accepted:
            self.connection.arducopter_disarm()
            accepted = self.cmd_ack()
        self.connection.motors_disarmed_wait()

    def takeoff(self, altitude: float):
        accepted = False
        while not accepted:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0, altitude)
            accepted = self.cmd_ack()

    def land(self):
        accepted = False
        while not accepted:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0)
            accepted = self.cmd_ack()

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
            print(f"Current position: ({msg.x}, {msg.y}, {msg.z})")

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

print("Launching Drone Control Script...")

vehicle = connect('127.0.0.1:60000', heartbeat_timeout=120, wait_ready=True)

while not vehicle.is_armable:
  print(" Waiting for vehicle to initialize...")
  time.sleep(1)

print("Arming motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
  print(" Waiting for arming...")
  time.sleep(1)

vehicle.simple_takeoff(10)
time.sleep(10)

msg = vehicle.message_factory.set_position_target_local_ned_encode(
          0,       # time_boot_ms (not used)
          0, 0,    # target system, target component
          mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
          0b0000111111000111, # type_mask (only speeds enabled)
          0, 0, 0, # x, y , z positions (not used)
          5, 0, 0, # x, y, z velocity in m/s
          0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
          0, 0,)   # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

t = 10 * 10 # 10 Hz

for i in range(t):
  vehicle.send_mavlink(msg)
  time.sleep(0.1)

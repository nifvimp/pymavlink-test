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

vehicle.simple_takeoff(5)
time.sleep(10)

def speed_msg(x, y, z):
  return vehicle.message_factory.set_position_target_local_ned_encode(
          0,       # time_boot_ms (not used)
          0, 0,    # target system, target component
          mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
          0b0000111111000111, # type_mask (only speeds enabled)
          0, 0, 0, # x, y , z positions (not used)
          x, y, z, # x, y, z velocity in m/s
          0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
          0, 0,)   # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

t = 10 * 10 # 10 Hz

for i in range(t):
  msg = speed_msg(5, 0, 0)
  vehicle.send_mavlink(msg)
  time.sleep(0.1)

for i in range(t):
  msg = speed_msg(0, 5, 0)
  vehicle.send_mavlink(msg)
  time.sleep(0.1)

for i in range(t):
  msg = speed_msg(-5, 0, 0)
  vehicle.send_mavlink(msg)
  time.sleep(0.1)

for i in range(t):
  msg = speed_msg(0, -5, 0)
  vehicle.send_mavlink(msg)
  time.sleep(0.1)

vehicle.send_mavlink(speed_msg(0, 0, 0))
time.sleep(5)

vehicle.mode = VehicleMode("RTL")

print("Close vehicle object")
vehicle.close()


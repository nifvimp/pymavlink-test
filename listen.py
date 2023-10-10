from pymavlink import mavutil
from constants import GSC_HOST as host
from constants import GSC_PORT as port

the_connection = mavutil.mavlink_connection('udpin:%u:%u' % (host, port))
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))
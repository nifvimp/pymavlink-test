# def guided():
#     connection.mav.set_mode_send(
#          connection.target_system,
#          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#          connection.mode_mapping()['GUIDED'])
#
#
# def stabilize():
#     connection.mav.set_mode_send(
#          connection.target_system,
#          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#          connection.mod_mapping()['STALIZE'])
#
#
# def arm():
#     connection.mav.command_long_send(
#         connection.target_system,
#         connection.target_component,
#         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#         0,
#         1, 0, 0, 0, 0, 0, 0)
#     connection.motors_armed_wait()
#
#
# def disarm():
#     connection.mav.command_long_send(
#         connection.target_system,
#         connection.target_component,
#         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#         0,
#         0, 0, 0, 0, 0, 0, 0)
#     connection.motors_disarmed_wait()
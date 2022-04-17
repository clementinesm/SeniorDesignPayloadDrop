# test getting parameters from the mavlink.

import time
import sys

from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

master.mav.param_request_list_send(
        master.target_system, master.target_component
)

master.mav.param_request_read_send(
        master.target_system, master.target_component, b'LAND_FLARE_ALT', -1
)

# --- this reads messages as they arrive! ---
message = master.recv_match(type='GPS_RAW_INT', blocking=True).to_dict()
print(message.keys())
print('current alt: %d' % (message['alt']))

time.sleep(5)
message = master.recv_match(type='GPS_RAW_INT', blocking=True).to_dict()
print(message.keys())
print('current alt: %d' % (message['alt']))


# reading GPS coordinates in a basic way (make more robust for flight)
#print(master.mav.__dict__)

# listing all the keys in the messages dict
print(master.messages.keys())

# I don't think this is the best way to read in messages
altitude = master.messages['GPS_RAW_INT'].alt
print(altitude)
print('time since gps message %d' % master.time_since('GPS_RAW_INT'))
#gps_info = master.messages['GPS_RAW_INT']
#print('time since gps message %d' % master.time_since('GPS_RAW_INT'))


#MAV_CMD_NAV_WAYPOINT 

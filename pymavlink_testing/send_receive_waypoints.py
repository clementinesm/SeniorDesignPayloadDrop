
# This file tests if we can send and receive waypoints to the pixhawk

import time
from pymavlink import mavutil, mavwp

# setup and wait for initial connection
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()
print("received heartbeat from system")

# generate waypoint object in mavlink
waypoints = mavwp.MAVWPLoader()
# not really sure what these do
seq = 1
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

# random gps coordinates, do not try to fly to these!!
lat = [30, 19, 11]
lon = [20, 28, 16]
alt = [2, 3, 4]

# add a sample waypoint to waypoints object - need to check format, might be outdated
for i in range(0, len(alt)):
	waypoints.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                         master.target_component,
                         seq,
                         frame,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 11, 0, 0,
                         lat[i],lon[i],alt[i]))
	seq += 1

# need to make sure that autocontinue is on for the waypoints!!

# send waypoint to pixhawk (shouldn't start going there)
master.waypoint_clear_all_send() # clear all of the current waypoints that are set
# check what the current waypoint is; there should be none I believe
curr_waypoint = master.waypoint_current()
print('current waypoint:')
print(curr_waypoint)



master.waypoint_count_send(waypoints.count()) # get pixhawk ready to receive wp.count() waypoints

# send each waypoint to the pixhawk; it will wait for each one
for i in range(waypoints.count()):
	msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
	master.mav.send(waypoints.wp(msg.seq))
	print('Sending waypoint {0}'.format(msg.seq))

# fetch current saved waypoints from the pixhawk and print them to the screen
# --- need to figure out which messages will do this ----

# trying to fetch waypoints through various methods
# --- postition_target method ---
msg = master.recv_match(type=['POSITION_TARGET_GLOBAL_INT'], blocking=True)
print(msg)
# --- mission_current method ---
msg = master.recv_match(type=['MISSION_CURRENT'], blocking=True)
print(msg)
# --- request list of mission items ---
# this requests all parameters (?)
master.mav.param_request_list_send(
        master.target_system, master.target_component
)
#msg = master.recv_match(type=['MISSION_REQUEST_LIST'])
#print(msg)


# get current waypoint
curr_waypoint = master.waypoint_current()
print('current waypoint:')
print(curr_waypoint)
# trying to set the current waypoint
master.waypoint_set_current_send(3)
time.sleep(1)
curr_waypoint = master.waypoint_current()
print('updated waypoint:')
print(curr_waypoint)
# get list of waypoints
waypoint_list = master.waypoint_request_list_send()
print('list of waypoints:')
print(waypoint_list)

# trying to set servo value through a given channel
master.set_servo(channel=7, pwm=1000) # check what pwm should be

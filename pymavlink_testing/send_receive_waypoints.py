
# This file tests if we can send and receive waypoints to the pixhawk

import time
from pymavlink import mavutil, mavwp

# generate a delay of a specific number of seconds (t)
def delay(t,seq,frame):
        msg = mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                         master.target_component,
                         seq,
                         frame,
                         mavutil.mavlink.MAV_CMD_CONDITION_DELAY,
                         0, 1, t, 0, 0, 0, 0, 0, 0)
        return msg

# setup and wait for initial connection
#master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
#master.wait_heartbeat()

# autodetect ports
ports = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])

for port in ports:
    print("%s" % port)

# try connecting to the specified port
master = mavutil.mavlink_connection(str(port), baud=921600)
master.wait_heartbeat()

print("received heartbeat from system")

# generate waypoint object in mavlink
waypoints = mavwp.MAVWPLoader()
# not really sure what these do
seq = 0 # was 1
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

# random gps coordinates, do not try to fly to these!!
lat = [30, 19, 11]
lon = [20, 28, 16]
alt = [2, 3, 4]
acceptance_radius = [36, 36, 36]

# add a sample waypoint to waypoints object - need to check format, might be outdated
for i in range(0, len(alt)):
	waypoints.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                         master.target_component,
                         seq,
                         frame,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 1, 0, 0, 0, 0,
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

print(waypoints.count())
# send each waypoint to the pixhawk; it will wait for each one
for i in range(waypoints.count()):
	msg = master.recv_match(type='MISSION_REQUEST',blocking=True)
	print('test')
	master.mav.send(waypoints.wp(msg.seq))
	print('Sending waypoint {0}'.format(msg.seq))

# note: need to have a 22 waypoint (home, I believe)
# why?: mission upload won't be accepted unless it is a valid mission!!

# send servo signal to the pixhawk
# 1100 - closed
# 1900 - open

# master.set_servo(channel=9, pwm=1100)
open_servo = mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                        master.target_component,
                        0,
                        frame,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        1,
                        9, 1100, 0, 0, 0, 0, 0)

# close the payload release system - might happen automatically?...
close_servo = mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                        master.target_component,
                        0,
                        frame,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        1,
                        9, 1900, 0, 0, 0, 0, 0)

# delay and upload servo commands separately
input("type any key to continue and upload servo commands")

# will this overwrite the commands
# might need to tell the mission planner how many commands it will receive?...
master.waypoint_count_send(3) # don't hardcode this now
master.recv_match(type=['MISSION_REQUEST'],blocking=True)
master.mav.send(open_servo) # sending automatically updates seq... need to better understand this
print("sending open_servo command")
seq+=1

# delay
master.recv_match(type=['MISSION_REQUEST'],blocking=True)
master.mav.send(delay(3,seq,frame))
print("sending servo delay command")
seq+=1

# close servo
master.recv_match(type=['MISSION_REQUEST'],blocking=True)
master.mav.send(close_servo)
print("sending close_servo command")
seq+=1

# send final waypoint to the pixhawk
# --- add a waypoint ----

# request download of ENTIRE mission plan
#master.mav.send(type=['MISSION_REQUEST_LIST']) # request all mission items
msg = master.waypoint_request_list_send() # doesn't seem to work
print('a')
print(msg)
# this and 'mission_item_int' aren't working, can't figure out why
msg = master.recv_match(type=['MISSION_COUNT'],blocking=True)

# check current mission count
print(msg)

#print('b')
#print(msg)
# iterate through each operation that we sent to the mission planner
for i in range(seq):
        # send the request for a specific mission item to the drone
        #master.mav.send(type=['MISSION_REQUEST_INT'], msg.seq) # need to check function call
        #master.mission_request_list()
	# receive each mission item from the drone
        msg = master.recv_match(type=['MISSION_ITEM_INT'],blocking=True)
        print(msg)
	# should be updating somehow; idk how

# ^ not sure if this will work
# mission_ack thing?

# need this to work; want to verify that the mission plan is uploaded successfully
print(msg)

# fetch current saved waypoints from the pixhawk and print them to the screen
# --- need to figure out which messages will do this ----


"""

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
"""

# comment/note: need to check if "send" will actually send wp to MP
# ^ might need another command to load waypoints in mp

#!/usr/bin/env python

# COE 374 - Senior Design - Spring 2022
# ADP drop team - Bradley, John, Kenneth, Nishcal, Rhea

# file that communicates with pixhawk and sends and receives flight data
# mavros is used to communite using mavlink protocols


import rospy
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix # not sure what this is

from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointPush

# load functions we have written
from airdop_functions import calculate_CARP, calc_approach_waypoints


# psuedocode/process


# run function - run this continuosly, activated by  boolean signal

# calculate CARP to determine drop point - save as drop waypoint

# using dubin's path, calculate the path to the target.
# pass the path as a list of waypoints to the mavlink
# pass final stretch of path as setpoints to control pitch/yall/roll
# recalculate the drop point using CARP
# **** pass drop point to the mavlink ******
# **** drop the payload when we get to the target coordinate *****

# wait a second or two

# repeat the entire process above until it is done 4 times!

# run everything through this script?...
# if __name__ == '__main__'


# things I don't know how to do:
# 1. continuously check if we have received the GPS coords of the target
# 2. organize the function/class structure of the code.
# 3. log all of our info to some output file so we can see what went wrong in the code. it will be easy to determine where our code went wrong (didn't drop adp, dropped at wrong location, etc.
# 4. analyze our performance beyond qualitative measures and measuring how far off the payload is from the target

# class that communicates with mavlink
class ADP_operation(self):
    # autoamtically setups up when it is created
    def __init__(self, *args):
        self.global_position = NavSatFix()
        # ROS services

        # get current GPS coordinate
        # some function here

        # srv
        self.push_waypoints = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        # below line is not correct fs
        self.push_setpoints = rospy.

        # how do we read these?
        # subscribers
        self.mission_waypoints = rospy.Subscriber('mavros/mission/waypoints', WaypointList)
        self.gps_coords = rospy.Subscriber('')

        # want to know waypoints and gps coordinates for now
        # ***would want more 

# do we need to set this to offboard mode? idk how this works

# should I create and pass a class that continuously updates position?
def run_adp_system():

    # create object (class) to communicate with the mavlink/pixhawk
    comms = ADP_operation() # this sets up connection to mavlink

    # section 1: wait for gps coordinates from target team
    target_found = False
    adp_drops = 0

    # execute the following while ros is set up
    while not rospy.is_shutdown():
        target_check_rate = rospy.Rate(2) # 2 Hz
        # ^ not sure what to do about this here
        while not target_found:
            # check the coordinates of the target, talk to both teams about this
            # below is just a placeholder function; talk to target recon team
            target_coords = check_for_target_coordinates() # GPS coords

            if target_coords is not None:
                target_found = True

        # the target is now found and we can now go through our cycle
        
        # might want to consider clearing waypoints here
        while adp_drops < 4:
            # step 1: carp calculation
            drop_coords = calculate_CARP(target_coords, wind_velo)
            # step 2: path planning/waypoint calculation
            curr_position = get_position() # use mavros protocol to get gps coordiantes of current locations
            waypoints = calc_approach(curr_position, drop_coords, wind_velo)
            
            # send waypoints using method in class
            comms.push_waypoints(start_index=0, waypoints=waypoints)
            # send setpoints to specify trajectory of aircraft near drop point
            # - need to calculate these setpoints as well


            # call payload drop function, waits until release coords are reached, then drops.

            adp_drops += 1 # only do this if it dropped succesfully!

        # ideally 

        # shut down the connection we have with the mavlink
        rospy.sleep() # check if this is how to do it

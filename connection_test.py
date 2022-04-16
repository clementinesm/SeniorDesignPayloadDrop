# Kenneth Meyer
# this tests the connection with the plane and sees if we can:
# 1. send and receive messages to the mavlink/pikhawk using subscribers
# 2. activate the servo intended to drop the ADP

import rospy
from pymavlink import mavutil
from mavros_msgs.msg import WaypointList, Altitude, ActuatorControl


# this class tests the connection using MAVROS
class connection_test(self):
    # autoamtically setups up when it is created
    def __init__(self, *args):
        #self.global_position = NavSatFix()
        
        # ROS services

        # srv - only need to activate the servo I believe
        #self.push_waypoints = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        #self.push_setpoints = rospy.ServiceProxy('mavros/mission/push')
        #self.clear_waypoints = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        # how do we read these?

        # service for sending actuator message to the missionplanner

        # subscribers
        self.altitude_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        #self.mission_waypoints_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList)
        #self.gps_global_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, global_position_callback)
        # uses a callback function to continuously update the data/access data from the pixhawk
        #self.mission_waypoints_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.local_position_callback)
        # want to know waypoints and gps coordinates for now
        # ***would want more 

    #
    # Callback functions - update object variables continuously
    #

    # callback functions that will allow us to access data from the pixhawk
    def altitude_callback(self, data):
        self.altitude = data

        # not really sure what this does tbh
        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True


    # test servo actuation
    def test_servo_drop(self):


    # write the altitude of the plane to a log file
    def log_altitude(self, rate_hz):
    	rate = rospy.rate(rate_hz)
    	rospy.loginfo("Testing altitude measurement")
    	rospy.loginfo("Current altitude = {0}".format(self.altitude))

    # check connection with the pixhawk & run servo actuation test
    def check_connection(self):

    	rospy.loginfo("testing pixhawk connection")


    	rospy.loginfo("checking servo actuation")
    	# need to come up with message for success and failure.

        


# this function tests connection using mavutil only
def run_pymav_tests():	
	# Start a connection listening to a UDP port; not sure what the port should be
	the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

	# Wait for the first heartbeat 
	#   This sets the system and component ID of remote system for the link
	the_connection.wait_heartbeat()
	# might want to figure out what to do if the connection fails & keeps looping



	print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

	# likely cannot visualize the output above, so save it to a file.
	file = open("output.txt","a")

	current_time = datetime.now()

	# set up write to file
	system = str(the_connection.target_system)
	component = str(the_connection.target_component)

	# write output to file
	message = "Heartbeat from system (system " + system + " component " + component + ")"

	file.write(message)

	# testing reading in messages
	try: 
	    altitude = the_connection.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
	    timestamp = the_connection.time_since('GPS_RAW_INT')
	    file.write(" GPS_RAW_INIT message received!!")

	except:
	    file.write("No GPS_RAW_INT message received")

	file.close()
	# purpose of this is to check if we can connect to mavlink with raspberry pi
	# still need to test this with MAVROS, as we will likely use it to actually fly

	# this checks if we can communicate with the servo

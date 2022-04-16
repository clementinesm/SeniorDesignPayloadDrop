# this is taken from the mavlink tutorial for pymavlink

# 

from pymavlink import mavutil
from datetime import datetime

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
# might want to figure out what to do if the connection fails & keeps looping



print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# likely cannot visualize the output above, so save it to a file.
file = open("/home/pi/payload_drop/SeniorDesignPayloadDrop/pymavlink_testing/output.txt","a")

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

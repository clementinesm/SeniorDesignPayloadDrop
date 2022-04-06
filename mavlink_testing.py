# COE 374 - Senior Design - 2022
# Payload Droppers

# not sure how to actually get this up and running tbh

# this file is to be used to test connecting mavlink and mission planner using pymavlink

from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages
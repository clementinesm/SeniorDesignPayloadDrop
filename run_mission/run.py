# Group 2 - payload drop team
# 4/24/2022
# executable for walking flight test on 

from pymalvink import mavutil
from mission_upload import run_mission_upload

# inputs: mission file
def run_mission(gps_coord):

    # set up connection to pixhawk
    # detect pixhawk port using serial usb connection
    ports = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])

    for port in ports:
        print("%s" % port)

    # try connecting to the specified port, there should only be one
    master = mavutil.mavlink_connection(str(port), baud=921600)
    master.wait_heartbeat()

    # calculate drop location using carp calculation (not needed)
    carp_coord = gps_coord # bc walking, drop on the target

    # generate a waypoint file (list of waypoints) from carp coord
    mission_file_name = generate_mission_file(carp_coord) # saves to payload_mission.txt

    # read the waypoints and send to mission planner!
    run_mission_upload(mission_file_name)


# JOHN! todo/what is psuedocode:
"""
    as of now, I haven't written anything that reads gps coords from a txt file
    I also haven't written a function that writes waypoints along the street outside of ASE ("generate_mission_file")

    I'm thinking that we have a function that writes the waypoints to a mission file so we have it saved somewhere, and then read them in using the other script in this folder. uploading a mission worked using the OG script in the "testing" folder, I removed the set_home function from the script because this will already happen/be done by the ASE team, and I removed the setup of the connection to the pixhawk from THAT file and moved it to this file as it makes more sense to establish the connection from this file. I haven't tested the run_mission_upload script in its modified state yet, it should work but there could be a typo or something - just wanted to give you a heads up.

    We could just call this run_mission function from within a script the target recognition team has so we never have to disconnect from pixhwk and reconnect using our script, that's probably the easiest way to go about doing this.
"""

if __name__ == "__main__":
    gps_coord = ___ # read in gps coordinates from text file
    run_mission(gps_coord) # read mission file with waypoints and servo commands

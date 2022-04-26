import airdrop_functions as adp
from airdrop_constants import *
from pymavlink import mavutil
from pymavlink import mavwp
import time

def run_mission(target_coord, master=None):

    # Get UAV Position
    uav_coord = np.zeros((2))
    uav_xy = adp.convert_coord_to_vector(uav_coord, target_coord)
    uav_xy = np.zeros((2))

    # Hard code wind
    windspeed = 5  # m/s
    wind_angle = -150*PI/180
    v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])

    # Calculate CARP
    carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE)

    # Generate waypoints
    first_pass_waypoints_xy = adp.calc_approach_waypoints_cw(uav_xy, carp_xy, APPROACH_ANGLE)
    turnaround_points_xy = adp.turnaround_cw(carp_xy, APPROACH_ANGLE)
    next_pass_waypoints_xy = adp.calc_approach_waypoints_cw(turnaround_points_xy[-1], carp_xy, APPROACH_ANGLE)
    next_pass_waypoints_xy = np.vstack((turnaround_points_xy, next_pass_waypoints_xy))

    first_pass_waypoints = adp.convert_vector_to_coord(first_pass_waypoints_xy, target_coord)
    next_pass_waypoints = [adp.convert_vector_to_coord(wp, target_coord) for wp in next_pass_waypoints_xy]
    next_pass_waypoints = np.array(next_pass_waypoints)

    # collect waypoints
    mission_array = adp.create_mission(first_pass_waypoints, next_pass_waypoints)

    if master==None:
        np.savetxt("./testing/payload_mission.txt", mission_array, fmt="%4d %4d %4d %4d %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %4d")
        adp.write_mission_file("./testing/payload_mission.waypoints", first_pass_waypoints, next_pass_waypoints)
        return

    # read the waypoints and send to mission planner
    wp = mavwp.MAVWPLoader()

    for mission_cmd in mission_array:
        ln_seq = int(mission_cmd[0])
        ln_current = int(mission_cmd[1])
        ln_frame = int(mission_cmd[2])
        ln_command = int(mission_cmd[3])
        ln_param1=float(mission_cmd[4])
        ln_param2=float(mission_cmd[5])
        ln_param3=float(mission_cmd[6])
        ln_param4=float(mission_cmd[7])
        ln_x=(float(mission_cmd[8]))
        ln_y=(float(mission_cmd[9]))
        ln_z=float(mission_cmd[10])
        ln_autocontinue = int(mission_cmd[11])

        mission_item_message = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                        ln_command,
                                                        ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
        wp.add(mission_item_message)
                
                    
    #cmd_set_home(home_location,home_altitude)
    # msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    # print(msg)
    # print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    # time.sleep(1)
    
    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
        # print(msg)
        master.mav.send(wp(msg.seq))
        #print(wp(msg.seq))
        # print('Sending waypoint {0}'.format(msg.seq))

if __name__ == "__main__":
    gps_coord = np.array([30.3247721,-97.6028609])   # Example
    run_mission(gps_coord)

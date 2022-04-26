import airdrop_functions as adp
from airdrop_constants import *
from pymavlink import mavutil
from pymavlink import mavwp

def run_mission(target_lat, target_lon, vehicle):
    """
    run_mission(target_lat, target_lon, master=None)\n
    Run a payload dropping mission. Writes mission to file/sends to pixhawk.

    Parameters
    ----------
        target_lat : float       
            lattitude of target
        target_lon : float       
            longitude of target        
        vehicle :  pymavlink object?
    """

    f = open("mission_outfile.txt", 'w')

    # Target 
    target_coord = np.array([target_lat, target_lon])

    ## TODO: Get UAV Position
    uav_lat = vehicle.location.global_relative_frame.lat
    uav_lon = vehicle.location.global_relative_frame.lon
    f.write("UAV location: {:f} {:f}\n".format(uav_lat, uav_lon))
    uav_coord = np.array([uav_lat, uav_lon])
    uav_xy = adp.convert_coord_to_vector(uav_coord, target_coord)

    # Hard code wind
    wind_angle = WIND_ANGLE
    windspeed = WIND_SPEED
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

    # Collect waypoints
    mission_array = adp.create_mission(first_pass_waypoints, next_pass_waypoints)
    np.savetxt("./testing/payload_mission.txt", mission_array, fmt="%4d %4d %4d %4d %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %4d")

    # format 
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

        mission_item_message = mavutil.mavlink.MAVLink_mission_item_message(vehicle.target_system, vehicle.target_component, ln_seq, ln_frame,
                                                        ln_command,
                                                        ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
        wp.add(mission_item_message)
    
    # send waypoints to autopilot
    vehicle.waypoint_clear_all_send()
    vehicle.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        msg = vehicle.recv_match(type=['MISSION_REQUEST'], blocking=True)
        f.write("Mission request\n")
        vehicle.mav.send(wp.wp(msg.seq))
        f.write("Waypoints sent\n")
    f.write("Done\n")
    f.close()

if __name__ == "__main__":
    # Mission test
    from dronekit import connect
    port = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])
    vehicle = connect(str(port[0]), wait_ready=True, baud=921600)
    gps_coord = np.array([30.3247721,-97.6028609])   # Example
    run_mission(gps_coord[0], gps_coord[1], vehicle)

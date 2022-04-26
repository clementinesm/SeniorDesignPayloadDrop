import airdrop_functions as adp
from airdrop_constants import *
import sys

import rospy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear, Waypointpull

def run_mission(target_lat, target_lon, uav_lat, uav_lon):
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
    # Target 
    target_coord = np.array([target_lat, target_lon])

    ##  Get UAV Position
    uav_coord = np.array([uav_lat, uav_lon])
    uav_xy = adp.convert_coord_to_vector(uav_coord, target_coord)

    # Hard code wind
    windspeed = WIND_SPEED
    wind_angle = WIND_ANGLE
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
    adp.write_mission_file("./testing/payload_mission.waypoints", first_pass_waypoints, next_pass_waypoints)

    # format 
    wp_push = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
    wp_clear = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
    wp_pull = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
    # not sure what "persistent" is.
    wps = []

    for mission_cmd in mission_array:
        wp = Waypoint()

        wp.is_current = int(mission_cmd[1])
        wp.frame = int(mission_cmd[2])
        wp.command = int(mission_cmd[3])
        wp.param1=float(mission_cmd[4])
        wp.param2=float(mission_cmd[5])
        wp.param3=float(mission_cmd[6])
        wp.param4=float(mission_cmd[7])
        wp.x_lat=(float(mission_cmd[8]))
        wp.y_long=(float(mission_cmd[9]))
        wp.z_alt=float(mission_cmd[10])
        wp.autocontinue = int(float(mission_cmd[11]))

        wps.append(wp)
    
    # send waypoints to autopilot
    wp_push(start_index=0, waypoints=wps)


if __name__ == "__main__":
    uav_lat = sys.argv[1]
    uav_lon = sys.argv[2]
    target_lat = sys.argv[3]
    target_lon = sys.argv[4]
    run_mission(target_lat, target_lon, uav_lat, uav_lon)

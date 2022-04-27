import airdrop_functions as adp
from airdrop_constants import *

def run_mission(target_coord, uav_coord):
    """
    run_mission(target_lat, target_lon)\n
    Run a payload dropping mission. Writes mission to file/sends to pixhawk.

    Parameters
    ----------
        target_lat : float       
            lattitude of target
        target_lon : float       
            longitude of target        
        vehicle :  pymavlink object?
    """

    ##  UAV Position
    uav_xy = adp.convert_coord_to_vector(uav_coord, target_coord)

    # Hard code wind
    wind_angle = WIND_ANGLE
    windspeed = WIND_SPEED
    v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])

    # Calculate CARP
    carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE)

    # Generate waypoints
    if APPROACH_ANGLE > 0:
        first_pass_waypoints_xy = adp.calc_approach_waypoints_cw(uav_xy, carp_xy, APPROACH_ANGLE)
        turnaround_points_xy = adp.turnaround_cw(carp_xy, APPROACH_ANGLE)
        next_pass_waypoints_xy = adp.calc_approach_waypoints_cw(turnaround_points_xy[-1], carp_xy, APPROACH_ANGLE)
        next_pass_waypoints_xy = np.vstack((turnaround_points_xy, next_pass_waypoints_xy))
    else:
        first_pass_waypoints_xy = adp.calc_approach_waypoints_ccw(uav_xy, carp_xy, APPROACH_ANGLE)
        turnaround_points_xy = adp.turnaround_ccw(carp_xy, APPROACH_ANGLE)
        next_pass_waypoints_xy = adp.calc_approach_waypoints_ccw(turnaround_points_xy[-1], carp_xy, APPROACH_ANGLE)
        next_pass_waypoints_xy = np.vstack((turnaround_points_xy, next_pass_waypoints_xy)) 

    first_pass_waypoints = adp.convert_vector_to_coord(first_pass_waypoints_xy, target_coord)
    next_pass_waypoints = [adp.convert_vector_to_coord(wp, target_coord) for wp in next_pass_waypoints_xy]
    next_pass_waypoints = np.array(next_pass_waypoints)

    # Write waypoints
    adp.write_mission_file("payload_mission.waypoints", first_pass_waypoints, next_pass_waypoints)

if __name__ == "__main__":
    target_coord = np.array([30.3246094,-97.6027463])
    
    uav_xy = 2*R_LOITER*np.array([np.sin(APPROACH_ANGLE), -np.cos(APPROACH_ANGLE)])
    uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
    print(uav_coord)
    print(APPROACH_ANGLE)
    run_mission(target_coord, uav_coord)

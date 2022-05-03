import airdrop_functions as adp
from matplotlib import pyplot as plt
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
    carp_25 = np.array([-8.23740238, 30.63770044])
    carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE)
    print(carp_xy)
    print(np.linalg.norm(carp_xy-carp_25)*3.28084, " ft")

    # Generate waypoints
    if APPROACH_ANGLE>0:
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

    # Read waypoints
    mission = np.loadtxt("payload_mission.waypoints", skiprows=1)
    waypoints = mission[mission[:,3]==16]
    waypoint_coords = waypoints[:,8:10]

    # Plot on map
    EPS = 0.0001
    FONTSIZE = 10
    BBox = (-97.607450, -97.598568, 30.320005, 30.327821)
    map_img = plt.imread("testing/map-earth.png")
    fig, ax = plt.subplots()

    ax.scatter(target_coord[1], target_coord[0], s=100, c='red', marker='*')
    ax.text(target_coord[1]-10*EPS, target_coord[0], 'target', size=FONTSIZE, color='white')

    ax.scatter(uav_coord[1], uav_coord[0], s=10, c='gold')
    ax.text(uav_coord[1]-5*EPS, uav_coord[0]+EPS, 'UAV', size=FONTSIZE, color='white')

    ax.scatter(waypoint_coords[:,1], waypoint_coords[:,0], s=10, c='lime')

    wind_coord = adp.convert_vector_to_coord(v_wind, [30.32164, -97.59908])
    plt.quiver(wind_coord[1], wind_coord[0], v_wind[0], v_wind[1], scale_units='inches', scale=5, color='white')
    ax.text(wind_coord[1]-2*EPS, wind_coord[0]+2*EPS, 'Wind', size=FONTSIZE, color='white')

    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])

    ax.imshow(map_img, zorder=0, extent = BBox, aspect= 'equal')
    plt.show()

if __name__ == "__main__":
    # TARGET COORDINATES
    target_coord = np.array([30.3236584366,-97.6019100])

    # Generate path
    uav_xy = 2*R_LOITER*np.array([np.cos(APPROACH_ANGLE), np.sin(APPROACH_ANGLE)])

    uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
    run_mission(target_coord, uav_coord)

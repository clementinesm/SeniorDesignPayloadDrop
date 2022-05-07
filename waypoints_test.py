import os
import numpy as np
import matplotlib.pyplot as plt
import airdrop_functions as adp
from airdrop_constants import *

def test_waypoints(target_coord, uav_coord, v_wind, fname):
    """
    run_mission(target_lat, target_lon)\n
    Run a payload dropping mission. Writes mission to file/sends to pixhawk.
    """

    ##  UAV Position
    uav_xy = adp.convert_coord_to_vector(uav_coord, target_coord)

    # Wind
    wind_angle = np.arctan2(v_wind[1],v_wind[0])

    # Generate waypoints
    if wind_angle>0:
        carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE_S)
        first_pass_waypoints_xy = adp.calc_approach_waypoints_cw(uav_xy, carp_xy, APPROACH_ANGLE_S)
        turnaround_points_xy = adp.turnaround_cw(carp_xy, APPROACH_ANGLE_S)
        next_pass_waypoints_xy = adp.calc_approach_waypoints_cw(turnaround_points_xy[-1], carp_xy, APPROACH_ANGLE_S)
        next_pass_waypoints_xy = np.vstack((turnaround_points_xy, next_pass_waypoints_xy))
    else:
        carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE_N)
        first_pass_waypoints_xy = adp.calc_approach_waypoints_ccw(uav_xy, carp_xy, APPROACH_ANGLE_N)
        turnaround_points_xy = adp.turnaround_ccw(carp_xy, APPROACH_ANGLE_N)
        next_pass_waypoints_xy = adp.calc_approach_waypoints_ccw(turnaround_points_xy[-1], carp_xy, APPROACH_ANGLE_N)
        next_pass_waypoints_xy = np.vstack((turnaround_points_xy, next_pass_waypoints_xy))   

    first_pass_waypoints = adp.convert_vector_to_coord(first_pass_waypoints_xy, target_coord)
    next_pass_waypoints = [adp.convert_vector_to_coord(wp, target_coord) for wp in next_pass_waypoints_xy]
    next_pass_waypoints = np.array(next_pass_waypoints)
    """
    # Write waypoints
    adp.write_mission_file("payload_mission.waypoints", first_pass_waypoints, next_pass_waypoints)

    # Read waypoints
    mission = np.loadtxt("payload_mission.waypoints", skiprows=1)
    waypoints = mission[mission[:,3]==16]
    waypoint_coords = waypoints[:,8:10]
    """

    # Plot on map
    EPS = 0.0001
    FONTSIZE = 10
    BBox = (-97.607450, -97.598568, 30.320005, 30.327821)
    map_img = plt.imread("map-earth.png")
    fig, ax = plt.subplots()

    ax.scatter(target_coord[1], target_coord[0], s=100, c='red', marker='*', label="Target")
    # ax.text(target_coord[1]-10*EPS, target_coord[0], 'target', size=FONTSIZE, color='white')

    ax.scatter(first_pass_waypoints[-1,1], first_pass_waypoints[-1:,0], s=100, c='lime', marker="*", label="CARP")
    ax.scatter(first_pass_waypoints[:-1,1], first_pass_waypoints[:-1:,0], s=10, c='lime', label="First Pass")
    ax.scatter(next_pass_waypoints[:,1], next_pass_waypoints[:,0], s=10, c='cyan', label="Second Pass")

    ax.scatter(uav_coord[1], uav_coord[0], s=10, c='orange', label="UAV")
    ax.text(uav_coord[1]+2*EPS, uav_coord[0], 'UAV', size=FONTSIZE, color='white')

    e_wind = v_wind/np.linalg.norm(v_wind)
    wind_coord = adp.convert_vector_to_coord(e_wind, [30.325, -97.6])
    plt.quiver(wind_coord[1], wind_coord[0], e_wind[0], e_wind[1], scale_units='inches', scale=2.5, color='white')
    ax.text(wind_coord[1]+2*EPS, wind_coord[0], 'Wind', size=FONTSIZE, color='white')

    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])
    lgnd = ax.legend()
    lgnd.legendHandles[0]._sizes = [100]
    lgnd.legendHandles[1]._sizes = [100]
    lgnd.legendHandles[2]._sizes = [30]
    lgnd.legendHandles[3]._sizes = [30]
    ax.imshow(map_img, zorder=0, extent = BBox, aspect= 'equal')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    plt.tight_layout()

    # Save fig
    if not os.path.exists("./figs"): os.mkdir("./figs")
    plt.savefig("./figs/"+fname)

    plt.show()



def main():
    test = 4

    if test==1:
        # NE approach
        target_coord = np.array([30.32445,-97.60234])
        uav_xy = D_APPROACH*np.array([-np.sin(APPROACH_ANGLE_N), np.cos(APPROACH_ANGLE_N)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 2  # m/s
        wind_angle = -70*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, uav_coord, v_wind, "N_approach.png")

    elif test==2:
        # NE approach, uav near approach point
        target_coord = np.array([30.32445,-97.60234])
        uav_xy = -D_APPROACH*np.array([np.cos(APPROACH_ANGLE_N), np.sin(APPROACH_ANGLE_N)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 2  # m/s
        wind_angle = -70*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, uav_coord, v_wind, "N_approach_close.png")

    elif test==3:
        # SW approach
        target_coord = np.array([30.32445,-97.60234])
        uav_xy = D_APPROACH*np.array([-np.sin(APPROACH_ANGLE_N), np.cos(APPROACH_ANGLE_N)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 2  # m/s
        wind_angle = 110*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, uav_coord, v_wind, "S_approach.png")

    elif test==4:
        # SW approach, uav near approach point
        target_coord = np.array([30.32445,-97.60234])
        uav_xy = -D_APPROACH*np.array([np.cos(APPROACH_ANGLE_S), np.sin(APPROACH_ANGLE_S)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 2  # m/s
        wind_angle = 110*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, uav_coord, v_wind, "S_approach_close.png")

    elif test==5:
        # Real drop test
        # NE approach
        target_coord = np.array([30.3247721,-97.6028609])
        uav_xy = 2*R_LOITER*np.array([-np.sin(APPROACH_ANGLE_S), np.cos(APPROACH_ANGLE_S)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 0  # m/s
        wind_angle = -150*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, uav_coord, v_wind)

    elif test==6:
        # Real drop test
        # NE approach
        target_coord = np.array([30.323960,-97.602418])
        uav_xy = 2*R_LOITER*np.array([-np.sin(APPROACH_ANGLE_S), np.cos(APPROACH_ANGLE_S)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 5.36  # m/s
        wind_angle = -110*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, uav_coord, v_wind)


if __name__=="__main__":
    main()
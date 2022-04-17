import numpy as np
import matplotlib.pyplot as plt

import airdrop_functions as adp
from airdrop_constants import *

def test_waypoints(target_coord, v_wind, uav_coord):

    # Convert uav coord to xy vector
    uav_xy = adp.convert_coord_to_vector(uav_coord, target_coord)

    # Calculate CARP and waypoints
    if target_coord[0] > HALFWAY_LAT:
        carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE_NW)
        waypoints = adp.calc_approach_waypoints_cw(uav_xy, carp_xy, APPROACH_ANGLE_NW)

        turnaround_points = adp.turnaround_cw(carp_xy, APPROACH_ANGLE_NW)
        waypoints_2 = adp.calc_approach_waypoints_cw(turnaround_points[-1], carp_xy, APPROACH_ANGLE_NW)
        next_points = np.vstack((turnaround_points, waypoints_2))
    else:
        carp_xy = adp.caclulate_CARP(v_wind, APPROACH_ANGLE_SE)
        waypoints = adp.calc_approach_waypoints_ccw(uav_xy, carp_xy, APPROACH_ANGLE_SE)

        turnaround_points = adp.turnaround_ccw(carp_xy, APPROACH_ANGLE_SE)
        waypoints_2 = adp.calc_approach_waypoints_ccw(turnaround_points[-1], carp_xy, APPROACH_ANGLE_SE)
        next_points = np.vstack((turnaround_points, waypoints_2))

    waypoint_coord = [adp.convert_vector_to_coord(wp, target_coord) for wp in waypoints]
    waypoint_coord = np.array(waypoint_coord)

    next_coord = [adp.convert_vector_to_coord(wp, target_coord) for wp in next_points]
    next_coord = np.array(next_coord)

    """
    File Format:
    ----------
    QGC WPL <VERSION>
    <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
    ----------
    """

    # Write to file
    with open("payload.waypoints", "w+") as ofile:
        ofile.write('QGC WPL 110\n')
        alt_ft = ALT*3.28084
        # Home
        ofile.write('%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%d\n' % (0,1,0,16,0,0,0,0,uav_coord[0],uav_coord[1],alt_ft,1))
        for i, wp in enumerate(waypoint_coord):
            ofile.write('%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%d\n' % (i+1,1,0,16,0,0,0,0,wp[0],wp[1],alt_ft,1))

    # Plot on map
    EPS = 0.0001
    FONTSIZE = 10
    BBox = (-97.607450, -97.598568, 30.320005, 30.327821)
    map_img = plt.imread("map-earth.png")
    fig, ax = plt.subplots()

    ax.scatter(target_coord[1], target_coord[0], s=10, c='red')
    ax.text(target_coord[1]-10*EPS, target_coord[0], 'target', size=FONTSIZE, color='white')

    ax.scatter(uav_coord[1], uav_coord[0], s=10, c='gold')
    ax.text(uav_coord[1]-5*EPS, uav_coord[0]+EPS, 'UAV', size=FONTSIZE, color='white')

    ax.scatter(waypoint_coord[:,1], waypoint_coord[:,0], s=10, c='lime')
    ax.scatter(next_coord[:,1], next_coord[:,0], s=10, c='cyan')

    wind_coord = [30.3254, -97.6]
    plt.quiver(wind_coord[1], wind_coord[0], v_wind[0], v_wind[1], scale_units='inches', scale=5, color='white')
    ax.text(wind_coord[1]-2*EPS, wind_coord[0]+2*EPS, 'Wind', size=FONTSIZE, color='white')

    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])

    ax.imshow(map_img, zorder=0, extent = BBox, aspect= 'equal')
    plt.show()

def main():
    test = 1

    if test==1:
        # NE approach
        target_coord = np.array([30.3247721,-97.6028609])
        uav_coord = np.array([30.324,-97.603288])
        windspeed = 2  # m/s
        wind_angle = -150*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, v_wind, uav_coord)
    elif test==2:
        # NE approach, uav near approach point
        target_coord = np.array([30.3247721,-97.6028609])
        uav_xy = -D_APPROACH*np.array([np.cos(APPROACH_ANGLE_NW), np.sin(APPROACH_ANGLE_NW)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 2  # m/s
        wind_angle = -150*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, v_wind, uav_coord)    
    elif test==3:
        # SW approach
        target_coord = np.array([30.3236654,-97.6022494])
        uav_coord = np.array([30.324,-97.603288])
        windspeed = 2  # m/s
        wind_angle = -150*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, v_wind, uav_coord)
    elif test==4:
        # SW approach, uav near approach point
        target_coord = np.array([30.3236654,-97.6022494])
        uav_xy = -D_APPROACH*np.array([np.cos(APPROACH_ANGLE_SE), np.sin(APPROACH_ANGLE_SE)])
        uav_coord = adp.convert_vector_to_coord(uav_xy, target_coord)
        windspeed = 2  # m/s
        wind_angle = -150*PI/180
        v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle), 0])
        test_waypoints(target_coord, v_wind, uav_coord)
if __name__=="__main__":
    main()
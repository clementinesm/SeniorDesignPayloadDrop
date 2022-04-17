import numpy as np
import matplotlib.pyplot as plt

import airdrop_functions as adp
from payload_constants import *

def test(target_coords, v_wind, uav_coords):
    # Origin of xy plane is target coords
    carp_xy = adp.caclulate_CARP(v_wind)
    carp_coords = adp.convert_vector_to_coords(carp_xy, target_coords)

    # Convert uav coords to xy vector
    uav_xy = adp.convert_coords_to_vector(uav_coords, target_coords)

    # Calculate waypoints
    loiter_center, approach_point = adp.calc_loiter_circle_center(uav_xy, carp_xy, v_wind)
    waypoints = adp.calc_approach_waypoints(uav_xy, loiter_center, approach_point)

    waypoint_coords = [adp.convert_vector_to_coords(wp, target_coords) for wp in waypoints]
    waypoint_coords = np.array(waypoint_coords)

    # Format waypoints
    """
    QGC WPL <VERSION>
    <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
    """

    with open("payload.waypoints", "w+") as ofile:
        ofile.write('QGC WPL 110\n')
        alt_ft = ALT*3.28084
        for i, wp in enumerate(waypoint_coords):
            if i==0:
                ofile.write('%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%d\n' % (i,1,0,16,0,0,0,0,wp[0],wp[1],alt_ft,1))
            else:
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (i,0,3,16,0,0,0,0,wp[0],wp[1],alt_ft,1))

    # Plot on map
    EPS = 0.0001
    FONTSIZE = 10
    BBox = (-97.60477, -97.59645, 30.32032, 30.32629)
    map_img = plt.imread("map.png")
    fig, ax = plt.subplots()

    ax.scatter(target_coords[1], target_coords[0], s=10, c='r')
    ax.text(target_coords[1], target_coords[0]+EPS, 'target', size=FONTSIZE)

    ax.scatter(uav_coords[1], uav_coords[0], s=10, c='g')
    ax.text(uav_coords[1], uav_coords[0]+EPS, 'UAV', size=FONTSIZE)

    ax.scatter(waypoint_coords[:,1], waypoint_coords[:,0], s=10)
    # ax.text(waypoint_coords[-1,1], waypoint_coords[-1,0]+EPS, 'p', size=FONTSIZE)

    ax.scatter(carp_coords[1], carp_coords[0], s=10)
    ax.text(carp_coords[1], carp_coords[0]+EPS, 'CARP', size=FONTSIZE)

    wind_coords = [30.3235, -97.599]
    plt.quiver(wind_coords[1], wind_coords[0], v_wind[0], v_wind[1], scale_units='inches', scale=2.5)
    ax.text(wind_coords[1]+5*EPS, wind_coords[0]-5*EPS, 'Wind', size=FONTSIZE)

    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])

    ax.imshow(map_img, zorder=0, extent = BBox, aspect= 'equal')
    plt.show()

def main():
    target_coords = np.array([30.3236654,-97.6022494])
    windspeed = 2    # m/s
    wind_angle = -150 * PI/180
    v_wind = np.array([windspeed*np.cos(wind_angle), windspeed*np.sin(wind_angle)])
    uav_coords = np.array([30.3247721,-97.6028609])

    test(target_coords, v_wind, uav_coords)

if __name__=="__main__":
    main()
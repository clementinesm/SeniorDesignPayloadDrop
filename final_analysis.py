import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import airdrop_functions as adp

def main(data):
    """
    Plot target, CARP, release points, release velocities, landings 
    
    """
    target_coord = np.array(data["target"])
    pred_target_coord = np.array(data["predicted_target"])
    CARP = np.array(data["CARP"])
    release_coords = np.array(data["release_coords"])
    speed = np.array(data["speed"])
    ground_course = np.array(data["ground_course"])
    landing_coords = np.array(data["landing_coords"])
    fname = data["fname"]

    # Distances
    print("Distance to target")
    for i, coord in enumerate(landing_coords):
        v = adp.convert_coord_to_vector(coord, target_coord)
        d = np.linalg.norm(v) * 3.28084
        print("Drop {:d}: {:f} ft".format(i, d))

    # Plot on map
    EPS = 0.0001
    FONTSIZE = 10
    BBox = (-97.607450, -97.598568, 30.320005, 30.327821)
    map_img = plt.imread("map-earth.png")
    fig, ax = plt.subplots()

    ax.scatter(target_coord[1], target_coord[0], s=50, c='navy', marker='s', label="Target")
    ax.scatter(pred_target_coord[1], pred_target_coord[0], s=50, facecolors='none', edgecolors='white', marker='s', label="Predicted Target")
    ax.scatter(CARP[1], CARP[0], s=100, c='orange', marker='*', label="CARP")
    ax.scatter(release_coords[:,1], release_coords[:,0], s=20, c='cyan', label="Release")
    ax.scatter(landing_coords[:,1], landing_coords[:,0], s=20, c='lime', label="Landing")


    radius = 40/3.28084/111111
    circ = Circle([target_coord[1], target_coord[0]], radius, fill=False, color="red", linestyle="--", linewidth=1.5, label="40ft")
    ax.add_patch(circ)

    for i in range(len(speed)):
        bearing = ground_course[i]*np.pi/180
        v_release = speed[i]*np.array([np.sin(bearing), np.cos(bearing)])
        plt.quiver(release_coords[i,1], release_coords[i,0], v_release[0], v_release[1], scale_units='inches', color='cyan', width=5e-3)

    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])
    lgnd = ax.legend()
    lgnd.legendHandles[0]._sizes = [100]
    lgnd.legendHandles[1]._sizes = [100]
    lgnd.legendHandles[2]._sizes = [200]
    lgnd.legendHandles[3]._sizes = [30]
    lgnd.legendHandles[4]._sizes = [30]
    ax.imshow(map_img, zorder=0, extent = BBox, aspect= 'equal')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    plt.tight_layout()

    # Save fig
    if not os.path.exists("./figs"): os.mkdir("./figs")
    plt.savefig("./figs/"+fname)

    plt.show()

if __name__=="__main__":

    riley_data = {
        "target":[30.323734283447266, -97.6020278930664],
        "predicted_target":[30.3236584366,-97.6019100],
        "CARP":[30.323869, -97.601999],
        "release_coords":[[30.3238996, -97.6019880],
                          [30.3234553, -97.6018042],
                          [30.3237607, -97.6019146]],
        "speed":[10.94, 10.13, 10.85],
        "ground_course":[159.19, 160.71, 159.77],
        "landing_coords":[[30.323680877685547, -97.6019058227539],
                          [30.3233699798584, -97.60203552246094],
                          [30.323833465576172, -97.6019515991211]],
        "fname":"riley_results.png"
    }

    frankie_data = {
        "target":[30.323734283447266, -97.6020278930664],
        "predicted_target":[],
        "CARP":[],
        "release_points":[],
        "speed":[],
        "ground_course":[],
        "landing_coords":[[30.323711395263672, -97.60203552246094],
                          [30.323631286621094, -97.60203552246094],
                          [30.323657989501953, -97.60198974609375]],
        "fname":"frankie_results.png"
    }

    main(riley_data)
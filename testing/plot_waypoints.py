import numpy as np
import matplotlib.pyplot as plt

# Hard-coded params
target_coord = np.array([30.3247721,-97.6028609])
v_wind = np.array([0.,0.,0.])
uav_coord = target_coord

# Read waypoints
mission = np.loadtxt("payload_mission.txt")
waypoints = mission[mission[:,3]==16]
waypoint_coords = waypoints[:,8:10]
print(waypoint_coords)

# Plot on map
EPS = 0.0001
FONTSIZE = 10
BBox = (-97.607450, -97.598568, 30.320005, 30.327821)
map_img = plt.imread("map-earth.png")
fig, ax = plt.subplots()

ax.scatter(target_coord[1], target_coord[0], s=100, c='red', marker='*')
ax.text(target_coord[1]-10*EPS, target_coord[0], 'target', size=FONTSIZE, color='white')

ax.scatter(uav_coord[1], uav_coord[0], s=10, c='gold')
ax.text(uav_coord[1]-5*EPS, uav_coord[0]+EPS, 'UAV', size=FONTSIZE, color='white')

ax.scatter(waypoint_coords[:,1], waypoint_coords[:,0], s=10, c='lime')

wind_coord = [30.3254, -97.6]
plt.quiver(wind_coord[1], wind_coord[0], v_wind[0], v_wind[1], scale_units='inches', scale=5, color='white')
ax.text(wind_coord[1]-2*EPS, wind_coord[0]+2*EPS, 'Wind', size=FONTSIZE, color='white')

ax.set_xlim(BBox[0],BBox[1])
ax.set_ylim(BBox[2],BBox[3])

ax.imshow(map_img, zorder=0, extent = BBox, aspect= 'equal')
plt.show()
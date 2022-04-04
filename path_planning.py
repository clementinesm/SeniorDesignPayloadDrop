import numpy as np
from matplotlib import patches
import matplotlib.pyplot as plt

def initial_approach(x, x_carp, v_wind):
    d = 500     # approach distance
    r = 150     # turn radius
    ppr = 12/(2*np.pi)  # points per radian
    e_wind = v_wind / np.linalg.norm(v_wind)  # unit wind vector

    p = x_carp + e_wind*d                     # final point in loiter
    e_ps = np.array([e_wind[1], -e_wind[0]])  # orthogonal to wind vector
    if np.cross(e_wind, e_ps) > 0:
        pass
    else:
        e_ps *= -1
    s = p + e_ps*r    # Center of loiter circle

    # Check if UAV is in near circle
    if np.linalg.norm(x-s) <= 2*r:
        p += d*e_wind
        s += d*e_wind
    else:
        pass

    theta = np.arctan2(s[1]-x[1],s[0]-x[0])        # angle to center of circle
    alpha = np.arcsin(r/np.linalg.norm(x-s))       # angle between center and tangent 
    D = np.abs(np.linalg.norm(x-s)*np.cos(alpha))  # Distance to tangent
    x_1 = x + np.array([D*np.cos(theta+alpha),D*np.sin(theta+alpha)])   # tangent point 1
    x_2 = x + np.array([D*np.cos(theta-alpha),D*np.sin(theta-alpha)])   # tangent point 2

    # Choose point to travel clockwise
    if np.cross(x_1-s, x_1-x) < 0:
        x_t = x_1
    else:
        x_t = x_2

    # Discrete points along circle
    beta_1 = np.arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (-2*np.pi)
    beta_2 = np.arctan2(p[1]-s[1], p[0]-s[0]) % (-2*np.pi)
    
    if beta_2-beta_1 > 0:
        angle = (beta_2-beta_1)%(-2*np.pi)
        npoints = int(np.ceil(abs(angle)*ppr))
        angles = np.linspace(0, (beta_2-beta_1)%(-2*np.pi), npoints) + beta_1
    else:
        angle = beta_2-beta_1
        npoints = int(np.ceil(abs(angle)*ppr))
        angles = np.linspace(0, beta_2-beta_1, npoints) + beta_1

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + r*np.cos(angles))
    waypoints[:,1] += (s[1] + r*np.sin(angles))

    return waypoints, beta_1, beta_2, s


# Test
x = np.array([-500., 0.])
x_carp = np.array([0.,0.])
v_wind = np.array([-1.,0.])
r = 150
waypoints, a1, a2, s = initial_approach(x, x_carp, v_wind)
print(waypoints)

circle = plt.Circle(s, r, color='b', linestyle="--", fill=False)
arc = patches.Arc(xy=s, width=2*r, height=2*r, angle=0, theta1=a2%(2*np.pi)*180/np.pi, theta2=a1%(2*np.pi)*180/np.pi)

fig, ax = plt.subplots()

ax.plot(x[0], x[1], 'ro')
ax.text(x[0]*1.05, x[1]*1.05, 'x', size=12)

ax.plot(s[0], s[1], 'ro')
ax.text(s[0]*1.05, s[1]*1.05, 's', size=12)

ax.scatter(waypoints[:,0], waypoints[:,1])
ax.text(waypoints[-1,0], waypoints[-1,1]+0.15, 'p', size=12)

xval = [x[0],waypoints[0,0]]
yval = [x[1],waypoints[0,1]]
ax.plot(xval,yval)

ax.add_patch(arc)
ax.set_xlim((-10,10))
ax.set_ylim((-10,10))
ax.axis("equal")
plt.show()
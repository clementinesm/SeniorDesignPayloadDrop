import numpy as np
from matplotlib import patches, widgets
import matplotlib.pyplot as plt



# CARP = [1,1,1]
# v_wind = [1,1,1]
# x_0 = [1,1,1]
# d = 100
# r = 50

# # center of circle 
# def calc_circle(carp, v_wind, d, r):
#     e_wind = v_wind/np.linalg.norm(v_wind)    
#     p = carp + d * v_wind / normalize_vector(v_wind)    
#     e_ps = np.array([e_wind[0], -e_wind[1]])
#     if np.cross(e_ps, e_wind) > 0:
#         pass
#     else:
#         e_ps *= -1

#     s = p + r*e_ps
#     return s

def calculate_waypoints(x, s, p):
    # Find tangent points
    r = np.linalg.norm(s-p)
    theta = np.arctan2(s[1]-x[1],s[0]-x[0])
    alpha = np.arcsin(r/np.linalg.norm(x-s))
    D = np.abs(np.linalg.norm(x-s)*np.cos(alpha))
    x_1 = x + np.array([D*np.cos(theta+alpha),D*np.sin(theta+alpha)])
    x_2 = x + np.array([D*np.cos(theta-alpha),D*np.sin(theta-alpha)])

    # Choose point to travel clockwise
    if np.cross(x_1-s, x_1-x) < 0:
        x_t = x_1
    else:
        x_t = x_2

    # Discrete points along circle
    beta_1 = np.arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (-2*np.pi)
    beta_2 = np.arctan2(p[1]-s[1], p[0]-s[0]) % (-2*np.pi)
    
    if beta_2-beta_1 > 0:
        angles = np.linspace(0, (beta_2-beta_1)%(-2*np.pi), 10) + beta_1
    else:
        angles = np.linspace(0, beta_2-beta_1, 10) + beta_1

    waypoints = np.zeros((10,2))
    waypoints[:,0] += (s[0] + r*np.cos(angles))
    waypoints[:,1] += (s[1] + r*np.sin(angles))

    return x_1, x_2, x_t, waypoints, beta_1, beta_2

# Test
x = np.array([0.,5.])
s = np.array([0.,0.])
p = np.array([-2.,2.])
r = np.linalg.norm(s-p)

x_1, x_2, x_t, waypoints, a1, a2 = calculate_waypoints(x, s, p)

circle = plt.Circle(s, r, color='b', linestyle="--", fill=False)

arc = patches.Arc(xy=s, width=2*r, height=2*r, angle=0, theta1=a2%(2*np.pi)*180/np.pi, theta2=a1%(2*np.pi)*180/np.pi)

fig, ax = plt.subplots()

ax.plot(x[0], x[1], 'ro')
ax.text(x[0]*1.05, x[1]*1.05, 'x', size=12)

# ax.plot(x_1[0], x_1[1], 'ro')
# ax.text(x_1[0]*1.05, x_1[1]*1.05, 'x_1', size=12)

# ax.plot(x_2[0], x_2[1], 'ro')
# ax.text(x_2[0]*1.05, x_2[1]*1.05, 'x_2', size=12)

# ax.plot(x_t[0], x_t[1], 'b*')
# ax.text(x_t[0]*1.05, x_t[1]*1.05, 'x_t', size=12)

ax.scatter(waypoints[:,0], waypoints[:,1])
xval = [x[0],waypoints[0,0]]
yval = [x[1],waypoints[0,1]]
ax.plot(xval,yval)

# ax.add_patch(circle)
ax.add_patch(arc)
ax.set_xlim((-10,10))
ax.set_ylim((-10,10))
ax.axis("equal")
plt.show()
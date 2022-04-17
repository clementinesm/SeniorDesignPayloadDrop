import numpy as np
from payload_constants import *

# Functions
def convert_coords_to_vector(coord, coord_ref):
    """
    coord     : terminal point
    coord_ref : coordinates of origin
    """
    lat_0 = coord_ref[0]
    lon_0 = coord_ref[1]
    lat_1 = coord[0]
    lon_1 = coord[1]
    x = (lon_1 - lon_0)*R_EARTH*(PI/180)*np.cos(lat_0 * PI/180)
    y = (lat_1 - lat_0)*R_EARTH*PI/180
    return np.array([x,y])

def convert_vector_to_coords(xy, coord_ref):
    """
    xy        : x-y displacement in meters
    coord_ref : coordinates of origin
    """
    lat_0 = coord_ref[0]
    lon_0 = coord_ref[1]
    lat_1 = lat_0 + (xy[1]/R_EARTH)*(180 / PI)
    lon_1 = lon_0 + (xy[0]/R_EARTH)*(180 / PI) / np.cos(lat_0 * PI/180)
    return np.array([lat_1, lon_1])

def determine_approach_direction(target_coords):
    if target_coords[0] < HALFWAY_LAT:
        heading_angle = -70*PI/180
        ccw = False
    else:
        heading_angle = 110*PI/180
        ccw = True  
    return

def caclulate_CARP(v_wind):

    theta = np.arctan2(v_wind[1], v_wind[0])   # wind angle

    # State vector
    x = 0.           # initial location of the payload
    y = 0.           # initial location of the payload
    z = ALT          # release height in meter(200 feet)
    vx = -V_UAV*np.cos(theta)      # x component of the payload velocity 
    vy = -V_UAV*np.sin(theta)      # y component of the payload velocity
    vz = 0.                        # z component of the payload velocity/ initially 0

    # Initial condition
    psi = np.array([x, y, z, vx, vy, vz])

    # Simulate Drop
    timestep = 0.01
    while psi[2] > 0:
        # Update wind estimate
        w = v_wind
        w = np.zeros(3)

        # Update airspeed v_r
        v_r = np.sqrt((psi[3]-w[0])**2 + (psi[4]-w[1])**2 + (psi[5]-w[2])**2) 

        # Time derivate of state vector
        psi_dot = np.zeros(6)
        psi_dot[0] = psi[3]  # v_x
        psi_dot[1] = psi[4]  # v_y
        psi_dot[2] = psi[5]  # v_z
        psi_dot[3] = -1/(2*M)*C_D*A*RHO*v_r*(psi[3]-w[0])   # a_x
        psi_dot[4] = -1/(2*M)*C_D*A*RHO*v_r*(psi[4]-w[1])   # a_y
        psi_dot[5] = -G-1/(2*M)*C_D*A*RHO*v_r*(psi[5]-w[2]) # a_z 

        # Increment one timestep
        psi += psi_dot*timestep   

    # return CARP (displacement from target in xy)
    return np.array([-psi[0], -psi[1]])

def caclulate_CARP_parallel(v_wind, heading_angle):

    # angle
    theta = heading_angle

    # State vector
    x = 0.           # initial location of the payload
    y = 0.           # initial location of the payload
    z = ALT          # release height in meter(200 feet)
    vx = -V_UAV*np.cos(theta)      # x component of the payload velocity 
    vy = -V_UAV*np.sin(theta)      # y component of the payload velocity
    vz = 0.                        # z component of the payload velocity/ initially 0

    # Initial condition
    psi = np.array([x, y, z, vx, vy, vz])

    # Simulate Drop
    timestep = 0.1
    while psi[2] > 0:
        # Update wind estimate
        w = v_wind

        # Update airspeed v_r
        v_r = np.sqrt((psi[3]-w[0])**2 + (psi[4]-w[1])**2 + (psi[5]-w[2])**2) 

        # Time derivate of state vector
        psi_dot = np.zeros(6)
        psi_dot[0] = psi[3]  # v_x
        psi_dot[1] = psi[4]  # v_y
        psi_dot[2] = psi[5]  # v_z
        psi_dot[3] = -1/(2*M)*C_D*A*P*v_r*(psi[3]-w[0])   # a_x
        psi_dot[4] = -1/(2*M)*C_D*A*P*v_r*(psi[4]-w[1])   # a_y
        psi_dot[5] = -G-1/(2*M)*C_D*A*P*v_r*(psi[5]-w[2]) # a_z 

        # Increment one timestep
        psi += psi_dot*timestep   

    # return CARP (displacement from target in xy)
    return np.array([-psi[0], -psi[1]])

def calc_loiter_circle_center(x_uav, x_carp, v_wind):
    e_wind = v_wind[0:2] / np.linalg.norm(v_wind[0:2])  # unit wind vector

    p = x_carp + e_wind*D_APPROACH            # final point in loiter
    e_ps = np.array([e_wind[1], -e_wind[0]])  # orthogonal to wind vector

    if np.cross(e_wind, e_ps) > 0:
        pass
    else:
        e_ps *= -1
    s = p + e_ps*R_LOITER    # Center of loiter circle

    # Check if UAV is near circle
    if np.linalg.norm(x_uav-s) <= 2*R_LOITER:
        p += D_APPROACH*e_wind
        s += D_APPROACH*e_wind

    return s, p

def calc_approach_waypoints(x_uav, s, p):

    theta = np.arctan2(s[1]-x_uav[1], s[0]-x_uav[0])           # angle to center of circle
    alpha = np.arcsin(R_LOITER/np.linalg.norm(x_uav-s))        # angle between center and tangent 
    D = np.abs(np.linalg.norm(x_uav-s)*np.cos(alpha))          # Distance to tangent
    x_1 = x_uav + np.array([D*np.cos(theta+alpha),D*np.sin(theta+alpha)])   # tangent point 1
    x_2 = x_uav + np.array([D*np.cos(theta-alpha),D*np.sin(theta-alpha)])   # tangent point 2

    # Choose point to travel counter-clockwise
    if np.cross(x_1-s, x_1-x_uav) < 0:
        x_t = x_1
    else:
        x_t = x_2

    # Discrete points along circle
    beta_1 = np.arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (-2*PI)
    beta_2 = np.arctan2(p[1]-s[1], p[0]-s[0]) % (-2*PI)
    
    if beta_2-beta_1 > 0:
        arc_angle = (beta_2-beta_1)%(-2*PI)
        npoints = int(np.ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = np.linspace(0, arc_angle, npoints) + beta_1
    else:
        arc_angle = beta_2-beta_1
        npoints = int(np.ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = np.linspace(0, arc_angle, npoints) + beta_1

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + R_LOITER*np.cos(angles))
    waypoints[:,1] += (s[1] + R_LOITER*np.sin(angles))

    return waypoints
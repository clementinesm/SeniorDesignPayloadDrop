import numpy as np

# Constants
R_EARTH = 6378*10**3    # radius of earth (m)
D_APPROACH = 150        # approach distance (m)
R_LOITER = 200                 # turn radius (m)
PPR = 15                # waypoints per radian 
M = 0.113398            # mass of the payload (kg)
C_D = 0.7               # drap coefficient of payload
A = 1                   # projected area of payload (m^2)
P = 1.225               # fluid density of air (kg/m^3)
G = 9.81                # gravitational constant (m/s^2)
ALT_GROUND = 0.8382     # ground altitude (m)
V_UAV = 15              # speed of UAV (m/s)


# Functions
def caclulate_CARP(target_coords, v_wind):
    # Input formatting
    target_coords = np.asfarray(target_coords)
    w_g = np.asfarray(v_wind)

    tx = target_coords[0]
    ty = target_coords[1]


    theta = np.arctan2(w_g[1], w_g[0])   # wind angle

    # State vector d(what we will iterate on)
    x = 0.           #initial location of the payload
    y = 0.           #initial location of the payload
    z = 60.69        #release height in meter(200 feet)
    vx = -V_UAV*np.cos(theta)      #the x component of the payload velocity 
    vy = -V_UAV*np.sin(theta)      #the y component of the payload velocity
    vz = 0.          #the z component of the payload velocity/ initially 0

    # Initial condition
    psi = np.array([x, y, z, vx, vy, vz])

    # Simulate Drop
    timestep = 0.1
    while psi[2] > 0:
        # Update wind estimate
        w = w_g

        # Update airspeed v_r
        v_r = np.np.sqrt((psi[3]-w[0])**2 + (psi[4]-w[1])**2 + (psi[5]-w[2])**2) 

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

    CARP_x = tx - psi[0]      #x component of the release point
    CARP_y = ty - psi[1]      #y component of the release point

    return np.array([CARP_x, CARP_y])

def add_meters_to_coord(coord, dx):
    lat_0 = coord[0]
    lon_0 = coord[1]
    lat_1 = lat_0 + (dx[0]/R_EARTH)*(180 / np.pi)
    lon_1 = lon_0 + (dx[1]/R_EARTH)*(180 / np.pi)/np.cos(lat_0 * np.pi/180)
    return np.array([lat_1, lon_1])

def convert_coord_to_vector(coord, coord_ref):
    lat_0 = coord_ref[0]
    lon_0 = coord_ref[1]
    lat_1 = coord[0]
    lon_1 = coord[1]
    x = (lat_1 - lat_0)*R_EARTH*np.pi/180
    y = (lon_1 - lon_0)*R_EARTH*(np.pi/180)*np.cos(lat_0 * np.pi/180)
    return np.array([x,y])

def calc_loiter_circle_center(x_carp, v_wind):
    e_wind = v_wind / np.linalg.norm(v_wind)  # unit wind vector

    p = x_carp + e_wind*D_APPROACH                     # final point in loiter
    e_ps = np.array([e_wind[1], -e_wind[0]])  # orthogonal to wind vector

    if np.cross(e_wind, e_ps) > 0:
        pass
    else:
        e_ps *= -1
    s = p + e_ps*R_LOITER    # Center of loiter circle

    return s, p

def calc_approach_waypoints(x_uav, x_carp, v_wind):

    e_wind = v_wind / np.linalg.norm(v_wind)  # unit wind vector

    p = x_carp + e_wind*D_APPROACH                     # final point in loiter
    e_ps = np.array([e_wind[1], -e_wind[0]])  # orthogonal to wind vector
    if np.cross(e_wind, e_ps) > 0:
        pass
    else:
        e_ps *= -1
    s = p + e_ps*R_LOITER    # Center of loiter circle

    # Check if UAV is in near circle
    if np.linalg.norm(x_uav-s) <= 2*R_LOITER:
        p += D_APPROACH*e_wind
        s += D_APPROACH*e_wind
    else:
        pass

    theta = np.arctan2(s[1]-x_uav[1],s[0]-x_uav[0])        # angle to center of circle
    alpha = np.arcsin(R_LOITER/np.linalg.norm(x_uav-s))       # angle between center and tangent 
    D = np.abs(np.linalg.norm(x_uav-s)*np.cos(alpha))  # Distance to tangent
    x_1 = x_uav + np.array([D*np.cos(theta+alpha),D*np.sin(theta+alpha)])   # tangent point 1
    x_2 = x_uav + np.array([D*np.cos(theta-alpha),D*np.sin(theta-alpha)])   # tangent point 2

    # Choose point to travel clockwise
    if np.cross(x_1-s, x_1-x_uav) < 0:
        x_t = x_1
    else:
        x_t = x_2

    # Discrete points along circle
    beta_1 = np.arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (-2*np.pi)
    beta_2 = np.arctan2(p[1]-s[1], p[0]-s[0]) % (-2*np.pi)
    
    if beta_2-beta_1 > 0:
        angle = (beta_2-beta_1)%(-2*np.pi)
        npoints = int(np.ceil(abs(angle)*PPR))
        angles = np.linspace(0, (beta_2-beta_1)%(-2*np.pi), npoints) + beta_1
    else:
        angle = beta_2-beta_1
        npoints = int(np.ceil(abs(angle)*PPR))
        angles = np.linspace(0, beta_2-beta_1, npoints) + beta_1

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + R_LOITER*np.cos(angles))
    waypoints[:,1] += (s[1] + R_LOITER*np.sin(angles))

    return waypoints
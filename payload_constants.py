import numpy as np

# Constants
A = 0.0324300504528       # projected area of payload (m^2)
ALT = 60.69               # flying altitude (m)
ALT_GROUND = 0.8382       # ground altitude (m)
C_D = 0.5                 # drap coefficient of payload
D_APPROACH = 100          # approach distance (m)
G = 9.81                  # gravitational constant (m/s^2)
HALFWAY_LAT = 30.324386   # approximate latitude of middle of field
M = 0.113398              # mass of the payload (kg)
PI = np.pi                # pi
PPR = 6                   # waypoints per revolution 
R_EARTH = 6378*10**3      # radius of earth (m)
R_LOITER = 60             # turn radius (m)
RHO = 1.225               # fluid density of air (kg/m^3)
RW_ANGLE = 110*PI/180     # runway orientation          
V_UAV = 15                # speed of UAV (m/s)
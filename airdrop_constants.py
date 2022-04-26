import numpy as np

# Constants
A = 0.0324300504528       # projected area of payload (m^2)
ALT = 60.69               # flying altitude (m)
ALT_GROUND = 0.8382       # ground altitude (m)
APPROACH_ANGLE_NW = 110*np.pi/180   # rad
APPROACH_ANGLE_SE = -70*np.pi/180   # rad
C_D = 0.                  # drap coefficient of payload
D_APPROACH = 100          # approach distance (m)
DROP_DELAY_TIME = 1.13    # delay from drop signal to servo activation
G = 9.81                  # gravitational constant (m/s^2)
HALFWAY_LAT = 30.324386   # approximate latitude of middle of field
M = 0.113398              # mass of the payload (kg)    
M_TO_FT = 3.28084         # m to ft conversion factor
PI = np.pi                # pi
PPR = 8                   # waypoints per revolution 
R_EARTH = 6378*10**3      # radius of earth (m)
R_LOITER = 75.            # turn radius (m)
RHO = 1.225               # fluid density of air (kg/m^3)        
V_UAV = 17                # speed of UAV (m/s)
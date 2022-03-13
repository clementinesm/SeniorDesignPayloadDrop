from math import sqrt
import numpy as np

#what we need as input
tx = 10         #x component of the target (lat)
ty = 20         #y component of the target (long)
wg = np.array([10, 20, 2])        #wind speed on the ground as a three dimensional vector

timestep = 0.01    #desired timestep
m = 10          #mass of the payload

zg = 1      #height where the wind is being measured 

# state vector d(what we will iterate on)
z = 60.69   #release height in meter(200 feet)
x = 0       #initial location of the payload
y = 0       #initial location of the payload
vx = 3     #the x component of the payload velocity/ initially equal to the velocity of the UAV
vy = 0      #the y component of the payload velocity/ initially 0
vz = 0      #the z component of the payload velocity/ initially 0

d = np.array([x, y, z, vx, vy, vz])


Cd = 0.1    #the coefficient of drag for the object/ determined experimentally/ we would need the drag coeffecient of the payload and the streamers. we might have to change this dynamically when the streamer is deployed
A = 1     #the projected area that is being acted on by the fluid/ determined by payload movement/ the bottom and front of the cylinder
p = 1.225  #the fluid density of air in kg/m^3
g = 9.81   #gravitational constant



while d[2] > 0:
    #update wind using wind power law
    wx = wg*(z/zg)**(1/7)
    #update Drag force
    Va = sqrt((d[3]-wx[0])**2+(d[4]-wx[1])**2+(d[5]-wx[2])**2)  #overall speed of the payload
    Fdx = -1/2*Cd*A*p*Va*(d[3]-wx[0])    #Drag force component in the x direction
    Fdy = -1/2*Cd*A*p*Va*(d[4]-wx[1])    #Drag force component in the y direction
    Fdz = -1/2*Cd*A*p*Va*(d[5]-wx[2])    #Drag force component in the z direction
    ddot = np.array([d[3], d[4], d[5], Fdx/m, Fdy/m, -g+Fdz/m]) #derivative of d vector
    d = d + ddot*timestep   #Euler method

releasepointx = tx - d[0]      #x component of the release point
releasepointy = ty - d[1]      #y component of the release point

print(releasepointx)
print(releasepointy)

#what we need for now: 
# drag coeffecient of the payload, mass of the payload
# how much above the ground is kestrel wind sensors
# projected area of the cylinder, 
# the altitude we will release in, 
# the speed of the UAV when we release
# what the wind and gps data would look like, so we can convert them to what we have
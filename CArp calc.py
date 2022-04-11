from math import sin, cos, sqrt
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15})


#what we need as input
tx = 10         #x component of the target (lat)
ty = 30         #y component of the target (long)
wg = np.array([-5, -10, 0.02])        #wind speed on the ground as a three dimensional vector

angle = np.arctan(wg[1]/wg[0])

timestep = 0.001    #desired timestep
m = 0.113398          #mass of the payload; kg

zg = 0.8382      #height where the wind is being measured 

# state vector d(what we will iterate on)
z = 60.69   #release height in meter(200 feet)
x = 0       #initial location of the payload
y = 0       #initial location of the payload
vx = 15.4333*cos(angle)      #the x component of the payload velocity 
vy = 15.4333*sin(angle)      #the y component of the payload velocity
vz = 0      #the z component of the payload velocity/ initially 0

d = np.array([x, y, z, vx, vy, vz])


Cd = 0.7    #the coefficient of drag for the object/ determined experimentally/ we would need the drag coeffecient of the payload and the streamers. we might have to change this dynamically when the streamer is deployed
A = 0.0324300504528     #the projected area that is being acted on by the fluid/ determined by payload movement/ the bottom and front of the cylinder
p = 1.225  #the fluid density of air in kg/m^3
g = 9.81   #gravitational constant



# while d[2] > 0:
#     #update wind using wind power law
#     wx = wg*(d[2]/zg)**(1/7)
#     #update Drag force
#     Va = sqrt((d[3]+wx[0])**2+(d[4]+wx[1])**2+(d[5]+wx[2])**2)  #overall speed of the payload
#     Fdx = -1/2*Cd*A*p*Va*(d[3]+wx[0])    #Drag force component in the x direction
#     Fdy = -1/2*Cd*A*p*Va*(d[4]+wx[1])    #Drag force component in the y direction
#     Fdz = -1/2*Cd*A*p*Va*(d[5]+wx[2])    #Drag force component in the z direction
#     ddot = np.array([d[3]+wx[0], d[4]+wx[1], d[5]+wx[2], Fdx/m, Fdy/m, -g+Fdz/m]) #derivative of d vector
#     d = d + ddot*timestep   #Euler method

d[3]=d[3]+wg[0]
d[4]=d[4]+wg[1]
d[5]=d[5]+wg[2]
while d[2] > 0:
    #update Drag force
    Va = sqrt((d[3])**2+(d[4])**2+(d[5])**2)  #overall speed of the payload
    Fdx = -1/2*Cd*A*p*Va*(d[3])    #Drag force component in the x direction
    Fdy = -1/2*Cd*A*p*Va*(d[4])    #Drag force component in the y direction
    Fdz = -1/2*Cd*A*p*Va*(d[5])    #Drag force component in the z direction
    ddot = np.array([d[3], d[4], d[5], Fdx/m, Fdy/m, -g+Fdz/m]) #derivative of d vector
    d = d + ddot*timestep   #Euler method


releasepointx = tx - d[0]      #x component of the release point
releasepointy = ty - d[1]      #y component of the release point

print(releasepointx)
print(releasepointy)

q1=plt.quiver(tx, ty, wg[0], wg[1], scale=200, color='r')
q2=plt.quiver(releasepointx, releasepointy, vx, vy, scale=200, color='b')
plt.quiverkey(q1, 1.2, 0.515, 2, 'Wind vector')
plt.quiverkey(q2, 1.2, 0.550, 2, 'UAV velocity')
plt.plot(tx, ty, marker="o", markersize=10, markerfacecolor="red")
plt.plot(releasepointx, releasepointy, marker="o", markersize=10, markerfacecolor="green")
plt.axis('scaled')
plt.xlim(0,20)
plt.ylim(20,40)
plt.title('Release point(green) calculation from target location(red) and wind direction')
plt.show()

#what we need for now: 
# drag coeffecient of the payload, mass of the payload
# how much above the ground is kestrel wind sensors
# projected area of the cylinder, 
# the altitude we will release in, 
# the speed of the UAV when we release
# what the wind and gps data would look like, so we can code in how to transform them into what we have
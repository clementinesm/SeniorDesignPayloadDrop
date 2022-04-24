# this file connects to the pixhawk and sends commands to open and close the servo

from pymavlink import mavutil

# connect to the pixhawk
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()
print("received heartbeat from system")

# --- servos must be armed in order to drop the payload ---

# checks if the servos are armed


# actuate the servos
master.set_servo(channel=9, pwm=1100) # opens the servo
#master.set_servo(channel=9, pwm=1900) # closes the servo

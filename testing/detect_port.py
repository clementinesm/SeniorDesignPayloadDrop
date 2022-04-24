# autodetects pixhawk serial port and connects

from pymavlink import mavutil
import time
# autodetect ports
ports = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])

for port in ports:
    print("%s" % port)

# try connecting to the specified port
connection = mavutil.mavlink_connection(str(port), baud=921600)
connection.wait_heartbeat()

# try a command to see what happens
connection.set_servo(channel=9, pwm=1500)
time.sleep(2)
connection.set_servo(channel=9, pwm=1900)

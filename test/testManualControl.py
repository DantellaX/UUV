# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print('Armed')

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].

while True:
    master.mav.manual_control_send(
  	master.target_system,
    	1000,
   	0,
    	500,
    	0,
   	0
    )
    print('x:' + str(master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('x')) + ' ' + 'y:' + str(master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('y')))
    time.sleep(1)

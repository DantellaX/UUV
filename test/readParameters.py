import time
import sys
import math

# Import mavutil
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

print('yaw: ' + str(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw'))))

print('x: ' + str(master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('x')))
print('y: ' + str(master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('y')))
print('z: ' + str(-master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('z')))

print('altitude: ' + str(master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('relative_alt')/1000.0))


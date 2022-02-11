import time
import sys
import math

# Import mavutil
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

message = master.recv_match(type='ATTITUDE', blocking=True).to_dict()
print('yaw: ' + str(message.get('yaw')*180/math.pi))


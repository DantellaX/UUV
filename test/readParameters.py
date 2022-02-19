import time
import sys
import math

# Import mavutil
from pymavlink import mavutil

g = 9.81 # m/s^2
rho_water = 997 # kg/m^3
pressure_air = 101325 # Pa


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

def getDepth():
    pressure = master.recv_match(type='SCALED_PRESSURE', blocking=True).to_dict().get('press_abs') * 100.0
    h = (pressure_air - pressure) /(rho_water * g)
    return str(round(h,3))

print('yaw: ' + str(round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw')), 3)))
'''
print('x: ' + str(master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('x')))
print('y: ' + str(master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('y')))
'''
print('altitude: ' + getDepth())
print('pressure: ' + str(round(master.recv_match(type='SCALED_PRESSURE', blocking=True).to_dict().get('press_abs') * 100.0, 3)))

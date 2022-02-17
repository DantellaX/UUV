import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase


def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    current_roll = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('roll'))
    current_pitch = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('pitch'))
    current_yaw = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw'))
    
    while round(current_roll,0) != roll or round(current_pitch,0) != pitch or round(current_yaw,0) != yaw:
    	master.mav.set_attitude_target_send(
	    int(1e3 * (time.time() - boot_time)), # ms since boot
	    master.target_system, master.target_component,
	    # allow throttle to be controlled by depth_hold mode
	    64,
	    # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
	    QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
	    0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
	    )
    	current_roll = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('roll'))
    	current_pitch = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('pitch'))
    	current_yaw = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw'))
    	print('roll: ' + str(current_roll))
    	print('pitch: ' + str(current_pitch))
    	print('yaw: ' + str(current_yaw))
    	time.sleep(1)
    print('Reach Target Attitude')
    
# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print('Armed')

master.set_mode('ALT_HOLD')
print('Depth Hold Mode')

set_target_attitude(0, 0, 180)


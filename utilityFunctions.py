from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time

gravity = 9.80665 # m/s^2
rhoWater = 997.0474 # kg/m^3
rhoSea = 1023.6 # kg/m^3
pressureAir = 101325 # Pa

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

    
# in Pa
def getPressure():
    pressure = 0.0
    for i in range(10):
    	pressure = pressure + round(master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict().get('press_abs') * 100.0, 4)
    return pressure / 10.0

# in meters
def getDepth():
    depth = (pressureAir- getPressure()) / (rhoWater * gravity)
    return round(depth, 4)
    # return round(master.recv_match(type='VFR_HUD', blocking=True).to_dict().get('alt'), 4)
    
# in degrees    
def getRoll():
    return round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('roll')), 4)
    
# in degrees    
def getPitch():
    return round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('pitch')), 4)
    
# in degrees    
def getYaw():
    return round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw')), 4)

# GPS_TYPE need to be MAV    
def setGPS():    
    time.sleep(0.2)
    master.mav.gps_input_send(
        0,  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        (8 |
         16 |
         32),
        0,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        0,  # Latitude (WGS84), in degrees * 1E7
        0,  # Longitude (WGS84), in degrees * 1E7
        getDepth(),  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s         
        0,  # GPS horizontal accuracy in m
        0,  # GPS vertical accuracy in m
        7   # Number of satellites visible.
    )

def test():
    while True:
        setGPS()
        print(master.recv_match(type='GPS_RAW_INT', blocking=True).to_dict().get('alt')/1000, getDepth())
        time.sleep(1)
        
def currentMode():
    value = master.wait_heartbeat().custom_mode
    if value == 0:
    	return 'STABILIZE'
    elif value == 1:
    	return 'ARCO'
    elif value == 2:
    	return 'ALT_HOLD'
    elif value == 3:
    	return 'AUTO'
    elif value == 4:
    	return 'GUIDED'
    elif value == 7:
    	return 'CIRCLE'
    elif value == 9:
    	return 'SURFACE'
    elif value == 16:
    	return 'PUSHOLD'
    elif value == 19:
    	return 'MANUAL'
    	
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print('Armed')
    
def disarm():
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Disarmed')   
    
def setMode(mode):
    master.set_mode(mode)
    print(mode)	

# x,y,r = [-1000,1000], z = [0,1000]
def manualControl(x,y,z,r):
    master.mav.manual_control_send(
  	master.target_system, x, y, z, r, 0)
    
# Sets the target attitude while in depth-hold mode. 'roll', 'pitch', and 'yaw' are angles in degrees.
def setTargetAttitude(roll, pitch, yaw):  
    arm()
    setMode('ALT_HOLD')
        
    while round(getRoll(),0) > roll + 5 or round(getRoll(),0) < roll - 5 or round(getPitch(),0) > pitch + 5 or \
    round(getPitch(),0) < pitch - 5 or round(getYaw(),0) > yaw + 5 or round(getYaw(),0) < yaw - 5:
    	master.mav.set_attitude_target_send(
	    int(1e3 * (time.time() - boot_time)), # ms since boot
	    master.target_system, master.target_component,
	    # allow throttle to be controlled by depth_hold mode
	    64,
	    # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
	    QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
	    0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
	    )
    	print('roll: ' + str(getRoll()) + ' ' + 'pitch: ' + str(getPitch()) + ' ' + 'yaw: ' + str(getYaw()))
    	time.sleep(1)
    print('Reach Target Attitude')

    
# Sets the target depth while in depth-hold mode.'depth' is technically an altitude, so set as negative meters below the surface.
def setTargetDepth(depth):
    arm()
    setMode('ALT_HOLD')
    
    while round(getDepth(),0) != depth:
        master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - boot_time)), # ms since boot
            master.target_system, master.target_component,
            coordinate_frame=5,
            type_mask=( # ignore everything except z position
                # 1 |
                # 2 |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                8 |
                16 |
                32 |
                64 |
                128 |
                256 |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                1024 |
                2048
            ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        print('depth: ' + str(getDepth()))
        time.sleep(1)
    print('Reach Target Depth')
    

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time

gravity = 9.80665 # m/s^2
rhoWater = 997.0474 # kg/m^3
rhoSea = 1023.6 # kg/m^3
pressureAir = 101325 # Pa
r = 6378137 # radius of earth in meters
c = r * 2 * math.pi # circumference of earth in meters

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

    
# in Pa
def getPressure():
    pressure = round(master.recv_match(type='SCALED_PRESSURE', blocking=True).to_dict().get('press_abs') * 100.0, 4)
    return pressure

# in meters
def getDepthPre():
    depth = (pressureAir- getPressure()) / (rhoWater * gravity)
    return round(depth, 4)

# in meters
def getDepthVFR():
    return round(master.recv_match(type='VFR_HUD', blocking=True).to_dict().get('alt'), 4)
    
# in degrees    
def getRoll():
    return round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('roll')), 4)
    
# in degrees    
def getPitch():
    return round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('pitch')), 4)
    
# in degrees    
def getYaw():
    return round(math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw')), 4)
    
# in degrees times 1e7   
def getLat():
    return master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('lat')
    
# in degrees times 1e7
def getLon():
    return master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('lon')

# in degrees
def meterToLat(x):
    lat = x / c * 360
    return lat

# in degrees
def meterToLon(y):
    lon = y * 360/ (c * math.cos(math.radians(getLat() / 10**7)))
    return lon

# in degrees
def latToMeter(lat):
    x = lat * c / 360
    return x

# in degrees
def lonToMeter(lon):
    y = lon * c * math.cos(math.radians(getLat() / 10**7)) / 360
    return y

def getSpeed():
    return master.recv_match(type='SCALED_IMU2', blocking=True).to_dict()
    
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print('Armed')
            
def disarm():
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Disarmed')   
     
def isArmed():
    try:
        return bool(master.wait_heartbeat().base_mode & 0b10000000)
    except:
        return False     
          
# Return current mode        
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
        
def setMode(mode):
    master.set_mode(mode)
    print(mode)	
    
def modeIs(mode):
    try:
        return bool(currentMode() == mode)
    except:
        return False
        
# x,y,r = [-1000,1000], z = [0,1000], x is forward, y is right, z is up.
def manualControl(x,y,z,r):
    master.mav.manual_control_send(
      master.target_system, x, y, z, r, 0)
    
# Set the target attitude while in depth-hold mode. 'roll', 'pitch', and 'yaw' are angles in degrees.
def setTargetAttitude(roll, pitch, yaw):  
    while not isArmed():
        arm()
    while not modeIs('ALT_HOLD'):
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
        print('roll: ', getRoll(), ' ', 'pitch: ', getPitch(), ' ', 'yaw: ', getYaw())
        time.sleep(1)
    print('Reach Target Attitude')

    
# Set the target depth while in depth-hold mode.'depth' is technically an altitude, so set as negative meters below the surface.
def setTargetDepth(depth):
    while not isArmed():
        arm()
    while not modeIs('ALT_HOLD'):
        setMode('ALT_HOLD')
    
    while round(getDepthPre(),0) != depth:
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
        print('depth: ', getDepthPre())
        time.sleep(1)
    print('Reach Target Depth')

# Set the target position while in depth-hold mode. Latitude and Longitude are in degrees.
def setGlobalPosition(lat, lon):
    while not isArmed():
        arm()
    while not modeIs('GUIDED'):
        setMode('GUIDED')
    
    while getLat() != lat or getLon() != lon:
        master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - boot_time)), # ms since boot
            master.target_system, master.target_component,
            coordinate_frame=5,
            type_mask=( # ignore everything except x,y position
                # 1 |
                # 2 |
                # 4 |
                8 |
                16 |
                32 |
                64 |
                128 |
                256 |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                1024 |
                2048
            ), lat_int=lat, lon_int=lon, alt=0, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        print('lat: ', getLat())
        print('lon: ', getLon())
        time.sleep(1)
    print('Reach Target Position')

# Set the target position while in depth-hold mode. Set as positive meters for north and east, negative meters for south and west.
def setLocalPosition(x, y):
    while not isArmed():
        arm()
    while not modeIs('GUIDED'):
        setMode('GUIDED')
     
    Xo = getLat()
    Yo = getLon()
    desLat = int(round(Xo + meterToLat(x) * 10**7, 0))
    desLon = int(round(Yo + meterToLon(y) * 10**7, 0))
    
    while round(0.1 * getLat(), 0) !=  round(0.1 * desLat, 0) or round(0.1 * getLon(), 0) != round(0.1 * desLon, 0):
        currentX = latToMeter((getLat() - Xo) / 10**7)
        currentY = lonToMeter((getLon() - Yo) / 10**7)
        print('lat: ', getLat(), 'x: ', currentX)
        print('lon: ', getLon(), 'y: ', currentY)
        print('heading: ', getYaw())

        master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - boot_time)), # ms since boot
            master.target_system, master.target_component,
            coordinate_frame=5,
            type_mask=( # ignore everything except x,y position
                # 1 |
                # 2 |
                # 4 |
                8 |
                16 |
                32 |
                64 |
                128 |
                256 |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                1024 |
                2048
            ), lat_int=desLat, lon_int=desLon, alt=0, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        time.sleep(1)
    print('Reach Target Position')
    
    


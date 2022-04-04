from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time

gravity = 9.80665 # m/s^2
densityWater = 997.0474 # kg/m^3
densitySea = 1023.6 # kg/m^3
pressureAir = 101325 # Pa
radiusEarth = 6378137 # radius of earth in meters
circumEarth = radiusEarth * 2 * math.pi # circumference of earth in meters

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

    
# in Pa
def getPressure():
    pressure = round(master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict().get('press_abs') * 100.0, 4)
    return pressure

# in meters
def getDepthPre():
    depth = (pressureAir- getPressure()) / (densityWater * gravity)
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

def getIMU():
    return master.recv_match(type='SCALED_IMU2', blocking=True).to_dict()
    
# in degrees times 1e7   
def getLat():
    return master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('lat')
    
# in degrees times 1e7
def getLon():
    return master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('lon')

# in degrees
def meterToLat(x):
    lat = x / circumEarth * 360
    return lat

# in degrees
def meterToLon(y):
    lon = y * 360/ (circumEarth * math.cos(math.radians(getLat() / 10**7)))
    return lon

# in degrees
def latToMeter(lat):
    x = lat * circumEarth / 360
    return x

# in degrees
def lonToMeter(lon):
    y = lon * circumEarth * math.cos(math.radians(getLat() / 10**7)) / 360
    return y

def getParam(param):
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        param.encode(),
        -1
    )
    return master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

def setParam(param, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param.encode(),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(param + ' param value set to: ', value)

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
    if value == 1:
        return 'ARCO'
    if value == 2:
        return 'ALT_HOLD'
    if value == 3:
        return 'AUTO'
    if value == 4:
        return 'GUIDED'
    if value == 7:
        return 'CIRCLE'
    if value == 9:
        return 'SURFACE'
    if value == 16:
        return 'PUSHOLD'
    if value == 19:
        return 'MANUAL'      

def setMode(mode):
    master.set_mode(mode)	
    if currentMode() == mode:
        print(mode)
    else:
        print('Fail to set ' + mode)    


# x,y,r = [-1000,1000], z = [0,1000], x is forward, y is right, z is up. Lowercap of r is 220.
def manualControl(x,y,z,r):
    while not isArmed():
        arm()
    master.mav.manual_control_send(master.target_system, x, y, z, r, 0)


# Set the target attitude while in depth-hold mode. 'roll', 'pitch', and 'yaw' are angles in degrees.
def setTargetAttitude(roll, pitch, yaw):  
    while not isArmed():
        arm()
    while currentMode() != 'ALT_HOLD':
        setMode('ALT_HOLD')    

    t = 3
    while getRoll() > roll + t or getRoll() < roll - t or getPitch() > pitch + t or \
    getPitch() < pitch - t or getYaw() > yaw + t or getYaw() < yaw - t:
        master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        64,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
        )
        print('roll: ', getRoll(), 'pitch: ', getPitch(), 'yaw: ', getYaw())
        time.sleep(1)


# Set the target yaw in degrees. yaw in [0,360].
def setTargetYaw(yaw):
    while not isArmed():
        arm()
    while currentMode() != 'ALT_HOLD':
        setMode('ALT_HOLD')

    t = 2
    currentYaw = getYaw() 
    if currentYaw < 0:
        currentYaw = 360 + currentYaw 
    while currentYaw > yaw + t or currentYaw < yaw - t: 
        error = yaw - currentYaw
        Kp = 2
        if error > 0:
            u = int(215 + Kp * error)
        if error < 0:
            u = int(-215 + Kp * error)
        if u > 1000:
            u = 1000
        if u < -1000:
            u = -1000
        manualControl(0,0,500,u)
        currentYaw = getYaw()
        if currentYaw < 0:
            currentYaw = 360 + currentYaw
        print('yaw: ', currentYaw)
    manualControl(0,0,500,0)
    

# Set the target depth while in depth-hold mode.'depth' is technically an altitude, so set as negative meters below the surface.
def setTargetDepth(depth):
    while not isArmed():
        arm()
    while currentMode() != 'ALT_HOLD':
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


# Set the target depth while in depth-hold mode via PID controller. Set as negative meters below the surface.
def setTargetDepthPID(depth):
    while not isArmed():
        arm()
    while currentMode() != 'ALT_HOLD':
        setMode('ALT_HOLD')

    while round(getDepthPre(),0) != depth:
        error = abs(depth - getDepthPre())
        if getDepthPre() > depth:
            Kp = 250
            u = int(500 - Kp * error) 
            if u < 0:
                u = 0
            manualControl(0,0,u,0)
    
        elif getDepthPre() < depth:
            Kp = 100
            u = int(500 + Kp * error) 
            if u > 1000:
                u = 1000
            manualControl(0,0,u,0)

        print('depth: ', getDepthPre())
        time.sleep(1)
    print('Reach Target Depth')


# Set the target position while in guided mode. Latitude and Longitude are in degrees times 1e7.
def setGlobalPosition(lat, lon):
    while not isArmed():
        arm()
    while currentMode() != 'GUIDED':
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


# Set the target position while in guided mode. Set as positive meters for north and east, negative meters for south and west.
def setLocalPosition(x, y):
    while not isArmed():
        arm()
 
    while currentMode() != 'GUIDED':
        setMode('GUIDED')

    Xo = getLon()
    Yo = getLat()
    
    desLon = int(round(Xo + meterToLon(x) * 10**7, 0))
    desLat = int(round(Yo + meterToLat(y) * 10**7, 0))
    
    while round(0.1 * getLat(), 0) !=  round(0.1 * desLat, 0) or round(0.1 * getLon(), 0) != round(0.1 * desLon, 0):
        currentX = lonToMeter((getLon() - Xo) / 10**7)
        currentY = latToMeter((getLat() - Yo) / 10**7)
        print('x: ', currentX)
        print('y: ', currentY)
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
    

# Set the target position via PID controller. Set as positive meters for north and east, negative meters for south and west.
def setLocalPositionPID(x, y):
    while not isArmed():
        arm()

    Xo = getLon()
    Yo = getLat()
    desLon = int(round(Xo + meterToLon(x) * 10**7, 0))
    desLat = int(round(Yo + meterToLat(y) * 10**7, 0))
    currentX = lonToMeter((getLon() - Xo) / 10**7)
    currentY = latToMeter((getLat() - Yo) / 10**7)

    t1 = 1
    while currentX > round(x, 0) + t1 or currentX < round(x, 0) - t1 or currentY > round(y, 0) + t1 or currentY < round(y, 0) - t1:
        currentX = lonToMeter((getLon() - Xo) / 10**7)
        currentY = latToMeter((getLat() - Yo) / 10**7)
        print('x: ', currentX)
        print('y: ', currentY)

        errorX = x - currentX
        errorY = y - currentY
        if errorY > 0:
            heading = math.degrees(math.atan(errorX / errorY))
        elif errorY < 0:
            if errorX > 0:
                heading = math.degrees(math.atan(errorX / errorY)) + 180
            elif errorX < 0:
                heading = math.degrees(math.atan(errorX / errorY)) - 180

        if heading < 0:
            heading = 360 + heading

        t = 2
        currentYaw = getYaw()
        if currentYaw < 0:
            currentYaw = 360 + currentYaw
        if currentYaw > heading + t or currentYaw < heading - t:
            manualControl(0,0,500,0)
            setTargetYaw(heading)

        errorDistance = math.sqrt(errorX**2 + errorY**2)
        Kp = 100
        u = int(Kp * errorDistance) 
        if u > 1000:
            u = 1000
        manualControl(u,0,500,0)

    print('Reach Target Position')
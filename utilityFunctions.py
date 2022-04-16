from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
import csv

GRAVITY = 9.80665 # m/s^2
RHO_WATER = 997.0474 # kg/m^3
RHO_SEA = 1023.6 # kg/m^3
PRESSURE_AIR = 101325 # Pa
RADIUS_EARTH = 6378137 # radius of earth in meters
CIRCUM_EARTH = RADIUS_EARTH * 2 * math.pi # circumference of earth in meters

class UUVcontrol():

    def __init__(self):
        # Create the connection
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.boot_time = time.time()
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

    # in Pa
    def get_pressure(self):
        pressure = round(self.master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict().get('press_abs') * 100.0, 4)
        return pressure

    # in meters
    def get_depth_pressure(self):
        depth = (PRESSURE_AIR-self.get_pressure()) / (RHO_WATER*GRAVITY)
        return round(depth, 4)

    # in meters
    def get_depth_VFR(self):
        return round(self.master.recv_match(type='VFR_HUD', blocking=True).to_dict().get('alt'), 4)
        
    # in degrees    
    def get_roll(self):
        return round(math.degrees(self.master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('roll')), 4)
        
    # in degrees    
    def get_pitch(self):
        return round(math.degrees(self.master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('pitch')), 4)
        
    # in degrees    
    def get_yaw(self):
        return round(math.degrees(self.master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw')), 4)

    def get_IMU(self):
        return self.master.recv_match(type='SCALED_IMU2', blocking=True).to_dict()
        
    # in degrees times 1e7   
    def get_lat(self):
        return self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('lat')
        
    # in degrees times 1e7
    def get_lon(self):
        return self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('lon')

    # in degrees
    def meter_to_lat(self, x):
        lat = x / CIRCUM_EARTH * 360
        return lat

    # in degrees
    def meter_to_lon(self, y):
        lon = y * 360/ (CIRCUM_EARTH * math.cos(math.radians(self.get_lat() / 10**7)))
        return lon

    # in degrees
    def lat_to_meter(self, lat):
        x = lat * CIRCUM_EARTH / 360
        return x

    # in degrees
    def lon_to_meter(self, lon):
        y = lon * CIRCUM_EARTH * math.cos(math.radians(self.get_lat() / 10**7)) / 360
        return y

    def get_param(self, param):
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            param.encode(),
            -1
        )
        return self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

    def set_param(self, param, value):
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            param.encode(),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        print(param + ' param value set to: ', value)

    def is_armed(self):
        try:
            return bool(self.master.wait_heartbeat().base_mode & 0b10000000)
        except:
            return False

    def arm(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print('Armed')
                
    def disarm(self):
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print('Disarmed')      
            
    # Return current mode        
    def current_mode(self):
        value = self.master.wait_heartbeat().custom_mode
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

    def set_mode(self, mode):
        self.master.set_mode(mode)	
        if self.current_mode() == mode:
            print(mode)
        else:
            print('Fail to set ' + mode)    


    # x,y,r = [-1000,1000], z = [0,1000], x is forward, y is right, z is up. Lowercap of r is 130.
    def manual_control(self, x, y, z, r):
        while not self.is_armed():
            self.arm()
        self.master.mav.manual_control_send(self.master.target_system, x, y, z, r, 0)

    '''
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
    '''

    # Set the target yaw in degrees. yaw in [0,360].
    def set_target_yaw(self, yaw):
        while not self.is_armed():
            self.arm()
        while self.current_mode() != 'MANUAL':
            self.set_mode('MANUAL')
        
        start = self.boot_time
        tt = time.localtime()
        now = time.strftime("%H:%M:%S", tt)
        data = open('Documents/UUV/data/' + 'yaw' + str(now), 'x')

        t = 5
        current_yaw = self.get_yaw() 
        if current_yaw < 0:
            current_yaw = 360 + current_yaw 
        while current_yaw > yaw + t or current_yaw < yaw - t: 
            error = yaw - current_yaw
            Kp = 0.5
            if error > 0:
                u = int(130 + Kp*error)
            if error < 0:
                u = int(-130 + Kp*error)
            if u > 1000:
                u = 1000
            if u < -1000:
                u = -1000
            self.manual_control(0, 0, 500, u)
            current_yaw = self.get_yaw()
            if current_yaw < 0:
                current_yaw = 360 + current_yaw
            print('yaw: ', current_yaw)
            
            end = time.time()
            data.write(str(end - start) + ', ' + str(self.get_yaw()) + '\n')

        data.close()
        self.manual_control(0, 0, 500, 0)
        
    '''
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
    '''

    # Set the target depth while in depth-hold mode via PID controller. Set as negative meters below the surface.
    def set_target_depth(self, depth):
        while not self.is_armed():
            self.arm()
        while self.current_mode() != 'MANUAL':
            self.set_mode('MANUAL')

        start = self.boot_time
        t = time.localtime()
        now = time.strftime("%H:%M:%S", t)
        data = open('Documents/UUV/data/' + 'depth' + str(now), 'a')

        current_depth = self.get_depth_pressure()
        while round(current_depth, 0) != depth:
            error = abs(depth - current_depth)
            if current_depth > depth:
                Kp = 250
                u = int(500 - Kp*error) 
                if u < 0:
                    u = 0
                self.manual_control(0, 0, u, 0)
        
            elif current_depth < depth:
                Kp = 125
                u = int(500 + Kp*error) 
                if u > 1000:
                    u = 1000
                self.manual_control(0, 0, u, 0)

            print('depth: ', current_depth)
            current_depth = self.get_depth_pressure()
            end = time.time()
            data.write(str(end - start) + ', ' + str(self.get_depth_pressure()) + '\n')
        data.close()
        print('Reach Target Depth')


    # Set the target position while in guided mode. Latitude and Longitude are in degrees times 1e7.
    def set_global_position(self, lat, lon):
        while not self.is_armed():
            self.arm()
        while self.current_mode() != 'GUIDED':
            self.set_mode('GUIDED')
        
        while self.get_lat() != lat or self.get_lon() != lon:
            self.master.mav.set_position_target_global_int_send(
                int(1e3 * (time.time() - self.boot_time)), # ms since boot
                self.master.target_system, self.master.target_component,
                coordinate_frame = 5,
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
                ), lat_int = lat, lon_int = lon, alt = 0, # (x, y WGS84 frame pos - not used), z [m]
                vx = 0, vy = 0, vz = 0, # velocities in NED frame [m/s] (not used)
                afx = 0, afy = 0, afz = 0, yaw = 0, yaw_rate = 0
                # accelerations in NED frame [N], yaw, yaw_rate
                #  (all not supported yet, ignored in GCS Mavlink)
            )
            print('lat: ', self.get_lat())
            print('lon: ', self.get_lon())
            time.sleep(1)
        print('Reach Target Position')


    # Set the target position while in guided mode. x:east-west, y:north-south. Set as positive meters for north and east, negative meters for south and west.
    def set_local_position_guided(self, x, y):
        while not self.is_armed():
            self.arm()
        while self.current_mode() != 'GUIDED':
            self.set_mode('GUIDED')

        x_origin = self.get_lon()
        y_origin = self.get_lat()
        
        des_lon = int(round(x_origin + self.meter_to_lat(x)*10**7, 0))
        des_lat = int(round(y_origin + self.meter_to_lat(y)*10**7, 0))
        
        while round(0.1 * self.get_lat(), 0) !=  round(0.1 * des_lat, 0) or round(0.1 * self.get_lon(), 0) != round(0.1 * des_lon, 0):
            current_x = self.lon_to_meter((self.get_lon()-x_origin) / 10**7)
            current_y = self.lat_to_meter((self.get_lat()-y_origin) / 10**7)
            print('x: ', current_x)
            print('y: ', current_y)
            print('heading: ', self.get_yaw())

            self.master.mav.set_position_target_global_int_send(
                int(1e3 * (time.time() - self.boot_time)), # ms since boot
                self.master.target_system, self.master.target_component,
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
                ), lat_int = des_lat, lon_int = des_lon, alt = 0, # (x, y WGS84 frame pos - not used), z [m]
                vx = 0, vy = 0, vz = 0, # velocities in NED frame [m/s] (not used)
                afx = 0, afy = 0, afz = 0, yaw = 0, yaw_rate = 0
                # accelerations in NED frame [N], yaw, yaw_rate
                #  (all not supported yet, ignored in GCS Mavlink)
            )
            time.sleep(1)
        print('Reach Target Position')
        

    # Set the target position via PID controller. x:east-west, y:north-south. Set as positive meters for north and east, negative meters for south and west.
    def set_local_position(self, x, y):
        while not self.is_armed():
            self.arm()
        while self.current_mode() != 'MANUAL':
            self.set_mode('MANUAL')

        start = self.boot_time
        tt = time.localtime()
        now = time.strftime("%H:%M:%S", tt)
        data = open('Documents/UUV/data/' + 'position' + str(now) , 'x')

        x_origin = self.get_lon()
        y_origin = self.get_lat()
        
        des_lon = int(round(x_origin + self.meter_to_lat(x)*10**7, 0))
        des_lat = int(round(y_origin + self.meter_to_lat(y)*10**7, 0))

        current_x = self.lon_to_meter((self.get_lon()-x_origin) / 10**7)
        current_y = self.lat_to_meter((self.get_lat()-y_origin) / 10**7)

        t1 = 1
        while current_x > round(x, 0) + t1 or current_x < round(x, 0) - t1 or current_y > round(y, 0) + t1 or current_y < round(y, 0) - t1:
            error_x = x - current_x
            error_y = y - current_y
            if error_y > 0:
                heading = math.degrees(math.atan(error_x / error_y))
            elif error_y < 0:
                if error_x > 0:
                    heading = math.degrees(math.atan(error_x / error_y)) + 180
                elif error_x < 0:
                    heading = math.degrees(math.atan(error_x / error_y)) - 180

            if heading < 0:
                heading = 360 + heading

            t = 5
            current_yaw = self.get_yaw()
            if current_yaw < 0:
                current_yaw = 360 + current_yaw
            if current_yaw > heading + t or current_yaw < heading - t:
                self.manual_control(0,0,500,0)
                self.set_target_yaw(heading)

            errorDistance = math.sqrt(error_x**2 + error_y**2)
            Kp = 100
            u = int(Kp * errorDistance) 
            if u > 1000:
                u = 1000
            self.manual_control(u,0,500,0)
            current_x = self.lon_to_meter((self.get_lon()-x_origin) / 10**7)
            current_y = self.lat_to_meter((self.get_lat()-y_origin) / 10**7)
            print('x: ', current_x)
            print('y: ', current_y)
            end = time.time()
            data.write(str(end - start) + ', ' + str(current_x) + ', ' + str(current_y) + '\n')

        data.close()
        print('Reach Target Position')

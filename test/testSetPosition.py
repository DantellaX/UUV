import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase

def set_target_position(target_x,target_y):

    current_x = master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('x')
    current_y = master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('y')
    
    while round(current_x,0) != target_x or round(current_y,0) != target_y:
        master.mav.set_position_target_local_ned_send(
            int(1e3 * (time.time() - boot_time)), # ms since boot
            master.target_system, master.target_component,
            coordinate_frame=1,
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
            ), x=target_x, y=target_y, z=0, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        current_x = master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('x')
        current_y = master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict().get('y')
        print('x: ' + str(current_x) + ' ' + str(current_y))
        time.sleep(1)
    print('Reach Target Position')

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print('Armed')

# set the desired operating mode'
master.set_mode('GUIDED')
print('Guided Mode')

# set a target position
set_target_position(10.0,10.0)

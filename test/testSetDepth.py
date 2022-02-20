import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase

def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    current_alt = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('relative_alt')/1000
    
    while round(current_alt,0) != depth:
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
        current_alt = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict().get('relative_alt')/1000
        print('alt: ' + str(current_alt))
        time.sleep(1)
    print('Reach Target Depth')


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print('Armed')

# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
master.set_mode(DEPTH_HOLD)
print('Depth Hold Mode')

# set a depth target
set_target_depth(-10.0)


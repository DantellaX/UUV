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
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        64,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )
    
# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print('Armed')

master.set_mode('MANUAL')
print('Mode set')

yaw_angle = roll_angle = pitch_angle = 0

for i in range(0, 360, 10):
    set_target_attitude(roll_angle, pitch_angle, i)
    time.sleep(1)
    yaw_angle = math.degrees(master.recv_match(type='ATTITUDE', blocking=True).to_dict().get('yaw'))
    print('yaw: ' + str(yaw_angle))


# clean up (disarm) at the end
master.arducopter_disarm()
master.motors_disarmed_wait()
print('Disarmed')


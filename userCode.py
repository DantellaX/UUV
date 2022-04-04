import utilityFunctions as uf
from pymavlink import mavutil
import time
import math

def connect():
    # Create the connection
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    boot_time = time.time()
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    return master

def main():
    uf.setLocalPositionPID(10,-10)
    #uf.setLocalPosition(2,2)
    #uf.setTargetDepthPID(-10)
    #uf.setTargetYaw(180)
        
    
if __name__ == '__main__':
    main()

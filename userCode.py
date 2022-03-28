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
    #print(uf.getSpeed())
    uf.setLocalPosition(10,-10)
    uf.setLocalPosition(-10,10)
    #uf.setGlobalPosition(338104590,-11893938642)
    #uf.setTargetDepth(-5)
    #while(True):
    #	print(uf.getDepthPre())
    
if __name__ == '__main__':
    main()

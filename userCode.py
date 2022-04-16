import utilityFunctions as uf
from pymavlink import mavutil
import time
import math

def main():
    task = uf.UUVcontrol()
    task.set_target_depth(-5)
    #task.set_local_position(10,10)
    #task.set_target_yaw(180)
        
    
if __name__ == '__main__':
    main()

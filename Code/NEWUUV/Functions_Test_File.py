from UUV_New_Functions import *
import time

# 1. 
while True:

    get_master() 

    get_heartbeat()

    get_latitude()

    get_longitude()

    get_heading() 

    get_pitch()

    get_yaw()

    get_roll() 

    get_pressure()

    get_distance()
    
    time.sleep(0.1)


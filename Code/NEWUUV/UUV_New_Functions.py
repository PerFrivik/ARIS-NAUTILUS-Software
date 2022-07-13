import pymavlink
from pymavlink import mavutil

#  get_master()         returns the master connection     

def get_master():
     master = mavutil.mavlink_connection("/dev/cu.usbmodem21101", baud=115200)
#   "/dev/cu.submodemXXXXX" Run UUV_Port_Finder.py to find the correct values for XXXXX, Pixhawk needs to be plugged in.
     return master

#  get_heartbeat()      returns the heartbeat that confirms the connection

def get_heartbeat():
    master = get_master()
    # Wait for a heartbeat before sending commands
    return master.wait_heartbeat()

#   get_message_interval()

def get_message_interval(message_id: int, frequency_hz: float):
    master = get_master()
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )
    
#   get_latitude()      returns latitude from gps sensor, negative when south

def get_latitude():
    master = get_master()
    # Configure ATTITUDE message to be sent at 1hz 
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)
    # The 

    AHRS2 = master.recv_match(type = 'AHRS2', blocking=True).to_dict()

    latitude = AHRS2.get("lat")

    print("this should be the latitude: {}".format(latitude))
    print(latitude)
    
    return latitude

#   get_longitude()     returns longitude from gps sensor, negative when west

def get_longitude():
    master = get_master()
    # Configure ATTITUDE message to be sent at 2Hz
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)

    AHRS2 = master.recv_match(type = 'AHRS2', blocking=True).to_dict()

    longitude = AHRS2.get("lng")

    print("this should be the longitude: {}".format(longitude))
    print(longitude)

    return longitude 

#   get_pressure()      returns pressure from barometer in bar (or pascal?)

def get_pressure():
    master = get_master()
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 2)

    SCALED_PRESSURE = master.recv_match(type = "SCALED_PRESSURE", blocking=True).to_dict()

    press_abs = SCALED_PRESSURE.get("press_abs")


    print("this should be the press_abs: {}".format(press_abs))
    print(press_abs)

    return press_abs


# STILL MISSING NEED TO TALK WITH ELECTRONICS 

#   get_heading()       returns angle relative to North from compass in radians

def get_heading():
    master = get_master()
    # Configure ATTITUDE message to be sent at 2Hz
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1)   
    VFR_HUD = master.recv_match(type = 'VFR_HUD', blocking=True).to_dict()

    heading = VFR_HUD.get("heading")

    print("this should be the heading: {}".format(heading))
    print(heading)
    
    return heading

    #CHANGE THIS USE #74 VFR_HUD!!!!!!! DONEEE BUT I DONT KNOW IF HE WANTS DEGREES OR WHAT HE WANTS FOR UNITS HE WANTS RAD `!!! 

#   get_pitch()         returns pitch value from PID controller

def get_pitch():
    master = get_master()
    # Configure ATTITUDE message to be sent at 2Hz
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)   
    AHRS2 = master.recv_match(type = 'AHRS2', blocking=True).to_dict()

    pitch = AHRS2.get("pitch")

    print("this should be the pitch: {}".format(pitch))
    print(pitch)
    
    return pitch

#   get_yaw()           returns yaw value from PID controller

def get_yaw():
    master = get_master()
    # Configure ATTITUDE message to be sent at 2Hz
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)
    AHRS2 = master.recv_match(type = 'AHRS2', blocking=True).to_dict()

    yaw = AHRS2.get("yaw")

    print("this should be the yaw: {}".format(yaw))
    print(yaw)
    
    return yaw

#   get_roll()          returns roll value from PID controller (Rick Roll)

def get_roll():
    master = get_master()
    # Configure ATTITUDE message to be sent at 2Hz
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)
    AHRS2 = master.recv_match(type = 'AHRS2', blocking=True).to_dict()

    roll = AHRS2.get("roll")

    print("this should be the roll: {}".format(roll))
    print(roll)
    
    return roll

#   get_leaksensors()   returns false if no leak is detected, true if leaky


# MISSINGGGG

def get_distance():
    master = get_master()
    get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 2)

    distance = master.recv_match(type = "DISTANCE_SENSOR", blocking=True).to_dict()

    print("this should be the press_abs: {}".format(distance))
    print(distance)

    return distance

#   get_sonardata()
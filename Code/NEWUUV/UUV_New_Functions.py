import pymavlink
from pymavlink import mavutil

####################################################
# Establishing a connection to the UUV             #
####################################################

def master_connection():
    # Start a connection listening to a UDP port
    master = mavutil.mavlink_connection('udpin:localhost:14540')

    # Wait for the first heartbeat 
    #   This sets the system and component ID of remote system for the link
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    # Once connected, use 'master' to get and send messages
    return master

####################################################
# Getting Pressure data                            #
####################################################

def Pressure_data(master):

    msg_pressure = master.recv_match(type='MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', blocking = True)
    return msg_Pressure_filter

####################################################
# Getting GPS data                                 #
####################################################

def GPS_data(master):

    msg_GPS = master.recv_match(type='MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', blocking = True)
    return msg_GPS_filter

####################################################
# Getting Compass data                            #
####################################################

def Compass_data(master):

    msg_Compass = master.recv_match(type='MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', blocking = True)
    return msg_Compass_filter  

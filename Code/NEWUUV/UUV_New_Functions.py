from pymavlink import mavutil

####################################################
# Establishing a connection to the UUV             #
####################################################

# Start a connection listening to a UDP port
master = mavutil.mavlink_connection('udpin:localhost:14540')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages

####################################################
# Getting data from the UUV                        #
####################################################

def data(master):
  
    master.mav.statustext_send(mavutil.mavlink.name)

####################################################
# Getting Pressure data                            #
####################################################

def pressure_data(master)

    msg_pressure = master.recv_match(type='MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', blocking = True)
    return msg_pressure

####################################################
# Getting GPS data                            #
####################################################

####################################################
# Getting Compass data                            #
####################################################

####################################################
# Getting data                            #
####################################################
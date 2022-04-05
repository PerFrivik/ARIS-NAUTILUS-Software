from doctest import master
import time
# Import mavutil
from pymavlink import mavutil

####################################################
# Function import test                             #
####################################################
def test_function():
    print("Hello NAUTILUS!")


####################################################
# Establishing a connection to the UUV             #
####################################################
def create_connection(): 
    # Create the connection
    #  If using a companion computer
    #  the default connection is available
    #  at ip 192.168.2.1 and the port 14550
    # Note: The connection is done with 'udpin (server/vehicle)' and not 'udpout (client/rasp)'.
    #  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
    #  uses a 'udpbcast' (client) and not 'udpin' (server).
    #  If you want to use QGroundControl in parallel with your python script,
    #  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
    #  E.g: --out udpbcast:192.168.2.255:yourport
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # Make sure the connection is valid
    master.wait_heartbeat()

####################################################
# Arming the UUV                                   #
####################################################
def ARM():
    # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM #

    # Arm
    # master.arducopter_arm() or:

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 
        
        0, 0, 0, 0, 0, 0) 

    #These last parameters dont contribute, since ARM_DISARM only takes two parameters

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

####################################################
# Disarming the UUV                                #
####################################################

def DISARM():

    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 
        
        0, 0, 0, 0, 0, 0)


####################################################
# Get some information !                           #
####################################################
def data():
    while True:
        try:
            print(master.recv_match().to_dict())
        except:
            pass
        time.sleep(0.1)
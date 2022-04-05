#Author(s): Per Frivik (pfrivik@student.ethz.ch)
#Date last change: 05.04.2022 

from doctest import master
import time
from pymavlink import mavutil
import sys
import math
from pymavlink.quaternion import QuaternionBase


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

    return(master)


####################################################
# Depth the UUV                                    #
####################################################

def set_target_depth(depth, boot, master_connection):
    master = master_connection
    boot_time = boot
        #  Sets the target depth while in depth-hold mode.

        # Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        # 'depth' is technically an altitude, so set as negative meters below the surface
        #     -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=( # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

####################################################
# Setting the attitude of the UUV                  #
####################################################

def set_target_attitude(roll, pitch, yaw, boot, master_connection):
    master = master_connection
    boot_time = boot

        #  Sets the target attitude while in depth-hold mode.

        # 'roll', 'pitch', and 'yaw' are angles in degrees.

    master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - boot_time)), # ms since boot
            master.target_system, master.target_component,
            # allow throttle to be controlled by depth_hold mode
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
        )


####################################################
# Arming the UUV                                   #
####################################################

def ARM(master_connection):
    master = master_connection

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

def DISARM(master_connection):
    master = master_connection

    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 
        
        0, 0, 0, 0, 0, 0)
    
    # wait until disarming confirmed
    print("Waitng for the vehicle to disarm")
    master.motors_disarmed_wait()
    print('Disarmed!')


####################################################
# Choosing Modes                                   #
####################################################

def Mode_selection(requested_mode, master_connection):
    master = master_connection

    # Choose a mode
    mode = requested_mode 

    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    # Set new mode
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # master.set_mode(mode_id) or:
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    # Not sure if we need this code!!!!!!
    # while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    #     master.set_mode(DEPTH_HOLD)

    #The following code only checks if the sent command is acknowledged by the UUV or not

    while True:
        # Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue

        # [Delete] if you see this, it means for real testing delete the code below and use the else
        # else:
        #     break

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

####################################################
# Get all vehicle paremeters (only for testing!)   #
####################################################

def Parameters(master_connection):
    master = master_connection

    # Request all parameters, Parameter list: https://www.ardusub.com/developers/full-parameter-list.html
    master.mav.param_request_list_send(
        master.target_system, master.target_component
    )
    while True:
        time.sleep(0.01)
        try:
            message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
            print('name: %s\tvalue: %d' % (message['param_id'],
                                        message['param_value']))
        except Exception as error:
            print(error)
            sys.exit(0)

####################################################
# Starting_procedure                               #
####################################################

def Starting_procedure(master_connection):
    master = master_connection

    boot_time = time.time()

    #Define wanted depth for start

    depth = 10

    set_target_depth(depth, boot_time, master)

    #Define wanted roll, pitch and yaw value for start
      
    roll = 10
    pitch = 10
    yaw = 10
    set_target_attitude(roll, pitch, yaw, boot_time, master)


    # arm ArduSub autopilot and wait until confirmed
    ARM(master)

    # set the desired operating mode
    Mode_selection('ALT_HOLD', master)

  

####################################################
# Get some information !                           #
####################################################

def data(master_connection):
    master = master_connection

    while True:
        try:
            print(master.recv_match().to_dict())
        except:
            pass
        time.sleep(0.1)


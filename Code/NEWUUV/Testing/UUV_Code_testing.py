

import time
# Import mavutil
from pymavlink import mavutil


# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection("/dev/cu.usbmodem1201", baud=115200)

# Make sure the connection is valid
print("Heartbeat from (system %u component %u)" % (master.target_system, master.target_component))

# Get some information !

while 1:
    msg1 = master.recv_match(blocking=True)
    print(msg1)
  



# try: 
#     altitude = master.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
#     timestamp = master.time_since('GPS_RAW_INT')
# except:
#     print('No GPS_RAW_INT message received')
from UUV_Functions import test_function, create_connection, ARM, DISARM, Mode_selection, data
from doctest import master
import time
from pymavlink import mavutil
import sys

#Test function import from UUV_Functions.py
test_function()

#Establishing a connection to the UUV
create_connection()

#Request Mode 0 MAV_MODE_PREFLIGHT	System is not ready to fly, booting, calibrating, etc. No flag is set.
Mode_selection('MAV_MODE_PREFLIGHT')

#Request Mode 216 MAV_MODE_GUIDED_ARMED	System is allowed to be active, under autonomous control, manual setpoint
Mode_selection('MAV_MODE_GUIDED_ARMED')







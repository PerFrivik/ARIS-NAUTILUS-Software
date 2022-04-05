from Code.UUV_Functions import Starting_procedure
from UUV_Functions import test_function, create_connection, Mode_selection, data, Starting_procedure
from doctest import master
import time
from pymavlink import mavutil
import sys
import math
from pymavlink.quaternion import QuaternionBase


#Test function import from UUV_Functions.py
test_function()

#Establishing a connection to the UUV
master = create_connection()

Starting_procedure(master)












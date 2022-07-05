import pymavlink
from pymavlink import mavutil
import time

#real value of buoyancymotor in simulator
realBuoyancy = 50

realPitch = 0

realRoll = 0

realDepth = 0

lasttime = time.time()

def DepthToPressure(depth):
    pressure = depth * 0.0978
    pressure += 1
    return pressure

def get_latitude():
    return 0

#   get_longitude()     returns longitude from gps sensor, negative when west

def get_longitude():
    return 0

#   get_pressure()      returns pressure from barometer in bar (or pascal?)
def get_pressure():
    global realDepth
    global realBuoyancy
    global lasttime
    nowtime = time.time()
    realDepth -= (nowtime - lasttime) * (realBuoyancy - 50) * 0.004
    lasttime = time.time()
    print(realDepth)
    return DepthToPressure(realDepth) 
    

#   get_heading()       returns angle relative to North from compass in radians

def get_heading():
    return 0

#   get_pitch()         returns pitch value from PID controller

def get_pitch():

    return 0

#   get_yaw()           returns yaw value from PID controller

def get_yaw():
    return 0

#   get_roll()          returns roll value from PID controller (Rick Roll)

def get_roll():
    return 0

#   get_leaksensors()   returns false if no leak is detected, true if leaky
def get_leaksensors():
    return False

def set_buoyancy(x):
    global realBuoyancy
    realBuoyancy = x

def set_pitch(x):
    realPitch = x

def set_roll(x):
    realRoll = x



# MISSINGGGG

#   get_sonardata()
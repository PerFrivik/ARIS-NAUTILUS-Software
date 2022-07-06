import pymavlink
from pymavlink import mavutil
import time
import matplotlib.pyplot as pl

#real value of buoyancymotor in simulator
realBuoyancy = 50

desiredBuoyancy = 50

BuoyancyStrokeTime = 17

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
    global desiredBuoyancy
    global BuoyancyStrokeTime
    nowtime = time.time()
    realDepth -= (nowtime - lasttime) * (realBuoyancy - 50) * 0.004
    if(realDepth < 0):
        realDepth = 0
    if(desiredBuoyancy > realBuoyancy):
        realBuoyancy += (nowtime - lasttime) / BuoyancyStrokeTime * 100
    else:
        realBuoyancy -= (nowtime - lasttime) / BuoyancyStrokeTime * 100      
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

#   get_roll()          returns roll value from PID controller

def get_roll():
    return 0

#   get_leaksensors()   returns false if no leak is detected, true if leaky
def get_leaksensors():   
    return False

def set_buoyancy(x):
    global desiredBuoyancy
    desiredBuoyancy = x

def set_pitch(value):
    global realPitch
    realPitch = value

def set_roll(x):
    realRoll = x



# MISSINGGGG

#   get_sonardata()
import pymavlink
from pymavlink import mavutil
import time
import matplotlib.pyplot as pl
import math
import numpy as np

#real value of buoyancymotor in simulator
realBuoyancy = 50

desiredBuoyancy = 50

BuoyancyStrokeTime = 17

realPitch = 0

realRoll = 0

realDepth = 0

lasttime = time.time()

realHeading = math.pi + math.pi / 4

realLatitude = 47.366407

realLongitude = 8.665069

GPSdecimal_to_meters = 111319.9

lat = []
long = []


def DepthToPressure(depth):
    pressure = depth * 0.0978
    pressure += 1
    return pressure

def get_latitude():
    global realLatitude
    return realLatitude

#   get_longitude()     returns longitude from gps sensor, negative when west

def get_longitude():
    global realLongitude
    return realLongitude

#   get_pressure()      returns pressure from barometer in bar (or pascal?)
def get_pressure():
    global lat
    global long
    global realLatitude
    global realLongitude
    global realRoll
    global realHeading
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
    if(realRoll < 0):
        realHeading += np.deg2rad(-realRoll) * (nowtime - lasttime) * 0.1
    else:
        realHeading -= np.deg2rad(realRoll) * (nowtime - lasttime) * 0.1
    component_latitude = math.sin(realHeading)
    component_longitude = math.cos(realHeading)
    realLatitude += (component_latitude * 0.2 / GPSdecimal_to_meters) * (nowtime - lasttime)
    realLongitude += (component_longitude * 0.2 / GPSdecimal_to_meters) * (nowtime - lasttime)
    lat.append(realLatitude)
    long.append(realLongitude)
    lasttime = time.time()
    #print(realHeading)
    return DepthToPressure(realDepth) 

    

#   get_heading()       returns angle relative to North from compass in radians

def get_heading():
    global realHeading
    return realHeading

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

def set_buoyancy(value):
    global desiredBuoyancy
    desiredBuoyancy = value

def set_pitch(value):
    global realPitch
    realPitch = value

def set_roll(value):
    global realRoll
    realRoll = value

def makeplots():
    global lat
    global long
    pl.plot(lat, long)
    pl.show()


# MISSINGGGG

#   get_sonardata()
import math
import time
from email.errors import MissingHeaderBodySeparatorDefect

from torch import not_equal
from UUV_New_Functions import *
#expected_functions:
# SensorDataFunctions: 
#   get_latitude()      returns latitude from gps sensor in decimal degrees
#   get_longitude()     returns longitude from gps sensor in decimal degrees
#   get_pressure()      returns pressure from barometer in bar (or pascal?)
#   get_heading()       returns angle relative to North from compass in radians
#   get_pitch()         returns pitch value from PID controller
#   get_yaw()           returns yaw value from PID controller
#   get_roll()          returns roll value from PID controller
#   get_leaksensors()   returns false if no leak is detected, true if leaky
#
#MotorControlFunctions:
#   set_buoyancy(value) Sends value to Pico motor control board
#   set_pitch(value)    Sends value to Jorit, possibly include encoding for pitch
#   set_roll(value)     Sends value to Jorit, possibly include encoding for roll
#   emergency_co2()     controls the container of co2 to release the gas

#CLASSES

#class for storing coordinates, 2 properties: latitude and longitude
#latitude denotes the northern GPS coordinate, negative when south
#longitude denotes the eastern GPS coordinate, negative when west
class GPS_coordinate:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
#function for updating paramters of class
    def update_position(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude    

#class for storing vehicle attitude, 4 properties: heading, pitch, yaw, roll
#heading denotes the angle between north and vehicle direction
#pitch denotes pitch of vehicle
#yaw denotes yaw of vehicle
#roll denotes roll of vehicle
class vehicle_attitude:
    def __init__(self, heading, pitch, yaw, roll):
        self.heading = heading
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
#function for updating parameters of class
    def update_attitude(self, heading, pitch, yaw, roll):
        self.heading = heading
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

#class for storing waypoint objects, 4 proprties: latitude, longitude, lower_depth, upper_depth
#latitude and longitude denote GPS coordinates and define the position of the waypoint
#lower_depth denotes the depth at which the UUV begins transition to rising in meters until the next waypoint is reached
#upper_depth denotes the depth at which the UUV begins transition to sinking in meters until the next waypoint is reached        
class waypoint:
    def __init__(self, latitude, longitude, lower_depth, upper_depth):
        self.latitude = latitude
        self.longitude = longitude
        if(self.lower_depth > 0):
            self.lower_depth = lower_depth
        else:
            self.lower_depth = 0
        if(self.upper_depth > 0):        
            self.upper_depth = upper_depth
        else:
            self.upper_depth = 0   

#FUNCTIONS

#function for checking the leaksensors on the UUV, and run emergency_procedure if neccessary
def check_leaksensors():
    if(get_leaksensors()):
        emergency_procedure()
        global mission_running
        mission_running = False

#function that is called when a leak is detected in the uuv
#Deploys co2 gas cannister and pumps water out of buoyancy module
#waits until reaching the surface and then ends mission
def emergency_procedure():
    starttime = time.time()
    emergency_co2()
    buoyancy_up()
    while(pressure_to_depth_freshwater(get_pressure()) > 0.1):
        time.sleep(1)    

#function that converts barometer data to depth in meters
#takes pressure in bars as input and outputs depth in meters
#pressure is assumed to include surface pressure
def pressure_to_depth_freshwater(pressure):
    pressure -= 1
    depth = pressure / 0.0978
    return depth

#function for setting the buoyancy motor position to 100, causing the UUV to go up
def buoyancy_up():
    set_buoyancy(100)
    global motor_buoyancy
    motor_buoyancy = 100

#function for setting the buoyancy motor position to 0, causing the UUV to go down
def buoyancy_down():
    set_buoyancy(0)
    global motor_buoyancy
    motor_buoyancy = 0

#Function to check if UUV is in radius of waypoint, returns Boolean value
#4 Input paramters: waypointradius, waypointcounter, waypoints, est_pos
#waypointradius denotes the radius around a waypoint that the UUv has to pass through
#waypointcounter denotes the amount of waypoints the UUV has already passed through
#waypoints denotes a list of GPS_coordinate instances which correspond to the predetermined waypoints
#est_pos is the estimated position of the UUV
def check_waypoint(est_pos):
    #distance is the distance of the UUV to the current waypoint in decimal GPS_coordinates
    distance = math.sqrt(((waypoints[waypoint_counter].latitude - est_pos.latitude) ** 2)
                + ((waypoints[waypoint_counter].longitude - est_pos.longitude) ** 2))            
    distance_in_meters = distance * GPSdecimal_to_meters            
    if(distance_in_meters < waypoint_radius):
        return True
    else:
        return False

#Constant that denotes the value used to convert decimal GPS represantation to meters
GPSdecimal_to_meters = 111319.9
#Time it takes for the buoyancy module to fully pump out all the water in seconds    
buoyancy_fullstroke_time = 18
#max depth of the vehicle, if it goes deeper, it cant return (including some margin)
max_depth = 50
#optimal pitch when the UUV is descending
descending_pitch = -72
#optimal pitch when the UUV is ascending
ascending_pitch = 72
#expected position of buoyancy motor ranging from 0(filled with water) to 100(empty), neutral=50
motor_buoyancy = 50
#maximum depth where we still can get GPS signal (needs to be determined by testing) in meters
maxGPS_aqquisition_depth = 0.1
#average movementspeed of UUV in meters per second
average_horizontalspeed = 0.17
#radius of the waypoints that the UUV has to pass through in meters
waypoint_radius = 5
#counts the number of waypoints vehicle has passed through
waypoint_counter = 0
#list of waypointObjects the UUV is supposed to follow
waypoints = []
#Declares that the mission is currently running
mission_running = True
    
def main():
    #upper and lower depth at which UUV begins transition from descending/ascending to the other one
    upper_depth = waypoints[waypoint_counter].upper_depth
    lower_depth = waypoints[waypoint_counter].lower_depth
    #estimated position that the uuv thinks it's at, gets updated when moving
    est_pos = GPS_coordinate(get_latitude(), get_longitude())
    #attitude (pitch, yaw, roll, heading) of vehicle
    attitude = vehicle_attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
    #depth of vehicle in meters
    depth = pressure_to_depth_freshwater(get_pressure())
    #expected position of roll of the UUV the motor is trying to achieve ranging from -90(left) to 90(right), neutral = 0
    expected_roll = 0
    #expected position of pitch of the UUV the motor is trying to achieve ranging from -90(bottom) to 90(top), neutral = 0
    expected_pitch = 0
    #boolean value that indicates if vehicle is descending or ascending
    descending = True
    #boolean value that indicates if vehicle is currently changing from ascending to descending or vice versa
    transitioning = False

    while(mission_running):
        #updating depth and attitude
        depth = pressure_to_depth_freshwater(get_pressure())
        attitude.update_attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
        #Updating est_pos at beginning of loop by GPS if high enough else use approximation
        if(depth < maxGPS_aqquisition_depth):
            est_pos.update_position(get_latitude(), get_longitude())
        else:
            component_longitude = math.cos(attitude.heading)
            component_latitude = math.sin(attitude.heading)
            est_pos.update_position(est_pos.latitude + (component_latitude * average_horizontalspeed / GPSdecimal_to_meters),
                                     est_pos.longitude + (component_longitude * average_horizontalspeed / GPSdecimal_to_meters))

        #Check if UUV is currently at waypoint
        if(check_waypoint(est_pos)):
            upper_depth = waypoints[waypoint_counter].upper_depth
            lower_depth = waypoints[waypoint_counter].lower_depth 
            waypoint_counter = waypoint_counter + 1           
            #if waypoint was the last one, return to surface
            if(waypoint_counter >= len(waypoints)):
                buoyancy_up()
                while(pressure_to_depth_freshwater(get_pressure()) > 0.1):
                    time.sleep(1)
                mission_running = False    
                break 
        #send commands to motors   
        if(descending and expected_pitch != descending_pitch):
            set_pitch(descending_pitch)
            expected_pitch = descending_pitch
        if(not descending and expected_pitch != ascending_pitch):
            set_pitch(ascending_pitch)
            expected_pitch = ascending_pitch
        if(descending and motor_buoyancy != 0):
            buoyancy_down()           
        if(not descending and motor_buoyancy != 100):
            buoyancy_up()

        #add roll logic

        #add descend/ascend switch logic
        if((descending and depth >= lower_depth) or (not descending and depth <= upper_depth) or (depth >= max_depth)):
            transitioning = True
            if(descending):
                buoyancy_up()
                starttime = time.time()
            else:
                buoyancy_down()
                starttime = time.time()
        while(transitioning):
            
            check_leaksensors()

        #test for leaks in vehicle    
        check_leaksensors()




main()
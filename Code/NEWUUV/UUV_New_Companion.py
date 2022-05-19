import math
import time
from email.errors import MissingHeaderBodySeparatorDefect
from UUV_New_Functions import master_connection, Pressure_data, GPS_data, Compass_data
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
#   buoyancy_up()       controls the motor of the buoyancy module to pump water out
#   buoyancy_down()     controls the motor of the buoyancy module to pump water in
#   emergency_co2()     controls the container of co2 to release the gas

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

#function that is called when a leak is detected in the uuv
#Deploys co2 gas cannister and pumps water out of buoyancy module
#waits until reaching the surface and then ends mission
def emergency_procedure():
    starttime = time.time()
    emergency_co2()
    while(time.time() - starttime < buoyancy_fullstroke_time):
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
    distance_in_meters = distance * 111319.9            
    if(distance_in_meters < waypoint_radius):
        return True
    else:
        return False

#Time it takes for the buoyancy module to fully pump out all the water in seconds    
buoyancy_fullstroke_time = 18
#maximum depth where we still can get GPS signal (needs to be determined by testing) in meters
maxGPS_aqquisition_depth = 0.1
#average movementspeed of UUV in meters per second
average_horizontalspeed = 0.17
#radius of the waypoints the UUV has to pass through in meters
waypoint_radius = 5
#counts the number of waypoints vehicle has passed through
waypoint_counter = 0
#list of GPS_coordinates the UUV is supposed to follow
waypoints = []
    
def main():

    #Declares that the mission is currently running
    mission_running = True
    #estimated position that the uuv thinks it's at, gets updated when moving
    est_pos = GPS_coordinate(get_latitude(), get_longitude())
    #attitude (pitch, yaw, roll, heading) of vehicle
    attitude = vehicle_attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
    #depth of vehicle in meters
    depth = pressure_to_depth_freshwater(get_pressure())

    while(mission_running):
        #test for leaks in vehicle
        if(get_leaksensors()):
            emergency_procedure()
            mission_running = False
            break
        #updating depth and attitude
        depth = pressure_to_depth_freshwater(get_pressure())
        attitude.update_attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
        #Updating est_pos at beginning of loop by GPS if high enough else use approximation
        if(depth < maxGPS_aqquisition_depth):
            est_pos.update_position(get_latitude(), get_longitude())
        else:
            component_longitude = math.cos(attitude.heading)
            component_latitude = math.sin(attitude.heading)
            est_pos.update_position(est_pos.latitude + component_latitude * average_horizontalspeed,
                                     est_pos.longitude + component_longitude * average_horizontalspeed)

        #Check if UUV is currently at waypoint
        if(check_waypoint(est_pos)):
            waypoint_counter = waypoint_counter + 1
            if(waypoint_counter >= len(waypoints)):
                starttime = time.time()
                while(time.time() - starttime < buoyancy_fullstroke_time):
                    buoyancy_up()
                while(pressure_to_depth_freshwater(get_pressure()) > 0.1):
                    time.sleep(1)
                mission_running = False    
                break 








main()
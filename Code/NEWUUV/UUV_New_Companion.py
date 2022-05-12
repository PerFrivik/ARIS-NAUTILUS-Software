import math
from email.errors import MissingHeaderBodySeparatorDefect
from UUV_New_Functions import master_connection, Pressure_data, GPS_data, Compass_data
#expected_functions: 
#   get_latitude()      returns latitude from gps sensor, negative when south
#   get_longitude()     returns longitude from gps sensor, negative when west
#   get_pressure()      returns pressure from barometer in bar (or pascal?)
#   get_heading()       returns angle relative to North from compass in radians
#   get_pitch()         returns pitch value from PID controller
#   get_yaw()           returns yaw value from PID controller
#   get_roll()          returns roll value from PID controller
#   get_leaksensors()   returns false if no leak is detected, true if leaky


#class for storing coordinates, 3 properties: north, east and depth
#latitude denotes the northern GPS coordinate, negative when south
#longitude denotes the eastern GPS coordinate, negative when west
#depth denotes the depth under surface level in meters
class GPS_coordinate:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def update_position(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude    

class vehicle_attitude:
    def __init__(self, heading, pitch, yaw, roll):
        self.heading = heading
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    def update_attitude(self, heading, pitch, yaw, roll):
        self.heading = heading
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

#function that converts barometer data to depth in meters
#takes pressure in bars as input and outputs depth in meters
#pressure is assumed to include surface pressure
def pressure_to_depth_freshwater(pressure):
    pressure -= 1
    depth = pressure / 0.0978
    return depth

#maximum depth where we still can get GPS signal (needs to be determined by testing) in meters
maxGPS_aqquisition_depth = 0.5
#average movementspeed of UUV in meters per second
average_movementspeed = 0.17
#radius of the waypoints the UUV has to pass through in meters
waypoint_radius = 5


def main():
    #Declares that the mission is currently running
    mission_running = True
    #estimated position that the uuv thinks it's at, gets updated when moving
    est_pos = GPS_coordinate(get_latitude(), get_longitude())
    #attitude (pitch, yaw, roll, heading) of vehicle
    attitude = vehicle_attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
    #depth of vehicle in meters
    depth = pressure_to_depth_freshwater(get_pressure)
    #list of GPS_coordinates the UUV is supposed to follow
    waypoints = []

    while(mission_running):
        #test for leaks in vehicle
        if(get_leaksensors):
            emergencyprocedure()
            mission_running = False
            break
        #updating depth and attitude
        depth = pressure_to_depth_freshwater(get_pressure)
        attitude.update_attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
        #Updating est_pos at beginning of loop by GPS if high enough else use approximation
        if(depth < maxGPS_aqquisition_depth):
            est_pos.update_position(get_latitude(), get_longitude)
        else:
            component_longitude = math.cos(attitude.heading)
            component_latitude = math.sin(attitude.heading)
            est_pos.update_position(est_pos.latitude + component_latitude * average_movementspeed,
                                     est_pos.longitude + component_longitude * average_movementspeed)






main()
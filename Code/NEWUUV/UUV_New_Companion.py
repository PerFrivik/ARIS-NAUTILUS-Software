from UUV_New_Functions import master_connection, Pressure_data, GPS_data, Compass_data, error
#expected_functions: 
#   get_latitude()      returns latitude from gps sensor, negative when south
#   get_longitude()     returns longitude from gps sensor, negative when west
#   get_pressure()      returns pressure from barometer in bar (or pascal?)
#   get_heading()       returns angle relative to North from compass
#   get_pitch()         returns pitch value from PID controller
#   get_yaw()           returns yaw value from PID controller
#   get_roll()          returns roll value from PID controller


#class for storing coordinates, 3 properties: north, east and depth
#latitude denotes the northern GPS coordinate, negative when south
#longitude denotes the eastern GPS coordinate, negative when west
#depth denotes the depth under surface level in meters
class GPS_coordinate:
    def __init__(self, latitude, longitude, depth):
        self.latitude = latitude
        self.longitude = longitude
        self.depth = depth

class attitude:
    def __init__(self, heading, pitch, yaw, roll):
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



def main():
    #Declares that the mission is currently running
    mission_running = True
    #estimated position that the uuv thinks it's at, gets updated when moving
    est_pos = GPS_coordinate(get_latitude(), get_longitude(), pressure_to_depth_freshwater(get_pressure))

    attitude = attitude(get_heading(), get_pitch(), get_yaw(), get_roll())
    #list of GPS_coordinates the UUV is supposed to follow
    waypoints = []

    error()



main()
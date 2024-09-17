import math
from droneControl import connect, get_global_position, get_heading, goto_waypoint, arm, VehicleMode, drone_takeoff
from math import radians, cos, sin, sqrt, atan2
import time

#leader
vehicle = connect('')
#follower1
slave1= connect('')

##### target location #########
target_lat = 19.1347626
target_lon = 72.9124027

####### RFD senders ###########
#data1= 
#data2= 
###############################

def relative_pos(lat, lon, distance, heading, follower_heading):
        # Convert heading to radians and calculate new heading
    EARTH_RADIUS = 6371000
    heading_rad = math.radians(heading)
    new_heading_rad = heading_rad + math.radians(follower_heading)
    
    # Convert latitude and longitude to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    # Calculate the new latitude and longitude
    delta_lat = distance * math.cos(new_heading_rad) / EARTH_RADIUS
    delta_lon = distance * math.sin(new_heading_rad) / (EARTH_RADIUS * math.cos(lat_rad))
    
    new_lat_rad = lat_rad + delta_lat
    new_lon_rad = lon_rad + delta_lon
    
    # Convert the new latitude and longitude back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)
    
    return new_lat, new_lon


def formation_row():
    ######## leader ######
    current_pos = get_global_position(vehicle)
    lat_lead = current_pos[0]
    lon_lead = current_pos[1]
    lead_yaw = get_heading(vehicle)
    print(lead_yaw)
    ######### follower1 ######
    yaw1 = 90
    dist1 = 20
    lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
    print(lat1, lon1)
    ##### send msg to slave1 drone #############

    goto_waypoint(slave1,lat1,lon1,0)

    ############################################


def distance_to_target(vehicle,target_lat,target_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - target_lat)
    dlon = radians(current_lon - target_lon)
    a = sin(dlat / 2)**2 + cos(radians(target_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters

#master
VehicleMode(vehicle,"GUIDED")
print("master in GUIDED mode")
# slave1
VehicleMode(slave1,"GUIDED")
print("slave1 in GUIDED mode")
time.sleep(1)

#master
arm(vehicle)
print("arming the master")
#slave1
arm(slave1)
print("arming the slave1")
time.sleep(5)

#master
drone_takeoff(vehicle,10)
print("taking off master")
#slave1
drone_takeoff(slave1,10)
print("taking off slave1")
time.sleep(10)

#target for master
goto_waypoint(vehicle,target_lat,target_lon,0)
##################

while True:

    formation_row()
    time.sleep(1)
    distance = distance_to_target(vehicle,target_lat,target_lon)
    print(f"Distance to Target: {distance} m.")
    if distance <=3:
        break

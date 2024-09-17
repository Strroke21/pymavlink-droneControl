import math
from droneControl import connect,get_global_position, get_heading, condition_yaw
import time


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


def formation_rectangle():
    ####### Leader Position ######
    current_pos = get_global_position(vehicle)
    lat_lead = current_pos[0]
    lon_lead = current_pos[1]
    lead_yaw = get_heading(vehicle)
    print(lead_yaw)
    ######## follower1 ##########
    yaw1 = 90
    dist1 = 20
    lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
    print(lat1, lon1)
    ###### follower2 ###########
    dist2 = 28.28 
    yaw2 = 135
    lat2, lon2 = relative_pos(lat_lead, lon_lead, dist2, lead_yaw, yaw2)
    print(lat2, lon2)
    ###### follower3 ##########
    dist3 = 28.28
    yaw3 = 225
    lat3, lon3 = relative_pos(lat_lead, lon_lead, dist3, lead_yaw, yaw3)
    print(lat3, lon3)
    ###### follower 4 ########
    dist4 = 20
    yaw4 = 270
    lat4, lon4 = relative_pos(lat_lead, lon_lead, dist4, lead_yaw, yaw4)
    print(lat4, lon4)

def formation_arrow():
    ######## leader ######
    current_pos = get_global_position(vehicle)
    lat_lead = current_pos[0]
    lon_lead = current_pos[1]
    lead_yaw = get_heading(vehicle)
    print(lead_yaw)
    ######### follower1 ######
    yaw1 = 135
    dist1 = 20
    lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
    print(lat1, lon1)
    ######## follower2 #######
    yaw2 = 135
    dist2 = 40
    lat2, lon2 = relative_pos(lat_lead, lon_lead, dist2, lead_yaw, yaw2)
    print(lat2, lon2)
    ######## follower3#######
    yaw3 = 225
    dist3 = 20
    lat3, lon3 = relative_pos(lat_lead, lon_lead, dist3, lead_yaw, yaw3)
    print(lat3, lon3)
    ######## follower4#######
    yaw4 = 225
    dist4 = 40
    lat4, lon4 = relative_pos(lat_lead, lon_lead, dist4, lead_yaw, yaw4)
    print(lat4, lon4)

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
    ######## follower2 #######
    yaw2 = 90
    dist2 = 40
    lat2, lon2 = relative_pos(lat_lead, lon_lead, dist2, lead_yaw, yaw2)
    print(lat2, lon2)
    ######## follower3#######
    yaw3 = 270
    dist3 = 20
    lat3, lon3 = relative_pos(lat_lead, lon_lead, dist3, lead_yaw, yaw3)
    print(lat3, lon3)
    ######## follower4#######
    yaw4 = 270
    dist4 = 40
    lat4, lon4 = relative_pos(lat_lead, lon_lead, dist4, lead_yaw, yaw4)
    print(lat4, lon4)





vehicle = connect('tcp:127.0.0.1:5763')
#condition_yaw(vehicle,7,1)
#time.sleep(5)
#formation_rectangle()
#formation_arrow()
formation_row()
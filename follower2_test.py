#!/usr/bin/env python3

import math
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2
import time
import pyudev
from sender_rfd900x_multi import send_rfd900x_mavlink_data
from receiver_rfd900x_multi_v2 import mavlink_receiver_rfd900x, mavlink_data_queue


def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
    	                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
def VehicleMode(vehicle,mode):

    modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE", "LAND"]
    if mode == modes[0]:
        mode_id = 0

    elif mode == modes[1]:
        mode_id = 1

    elif mode == modes[2]:
        mode_id = 2

    elif mode == modes[3]:
        mode_id = 3
    
    elif mode == modes[4]:
        mode_id = 4

    elif mode == modes[5]:
        mode_id = 5

    elif mode == modes[6]:
        mode_id = 6

    elif mode == modes[7]:
        mode_id = 7

    elif mode == modes[8]:
        mode_id = 9

    else:
        mode_id = 12
    ##### changing to guided mode #####
    #mode_id = 0:STABILIZE, 1:ACRO, 2: ALT_HOLD, 3:AUTO, 4:GUIDED, 5:LOITER, 6:RTL, 7:CIRCLE, 9:LAND
    vehicle.mav.set_mode_send(
        vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
    
def drone_takeoff(vehicle, altitude):
    
    # Send MAVLink command to takeoff
    vehicle.mav.command_long_send(
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,                          # confirmation
        0,                          # param1 (min pitch, not used)
        0,                          # param2 (empty for now, not used)
        0,                          # param3 (empty for now, not used)
        0,                          # param4 (yaw angle in degrees, not used)
        0,                          # param5 (latitude, not used)
        0,                          # param6 (longitude, not used)
        altitude                    # param7 (altitude in meters)
    )

def get_global_position(vehicle):
    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    100,1)
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat/1e7 # lat
    lon = msg.lon/1e7 # lon
    alt = msg.alt/1000  # alt
    return [lat,lon,alt]

def goto_waypoint(vehicle,latitude, longitude, altitude):
    msg = vehicle.mav.set_position_target_global_int_encode(
        time_boot_ms=10,
        target_system=vehicle.target_system,       # Target system (usually 1 for drones)
        target_component=vehicle.target_component,    # Target component (usually 1 for drones)
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame of reference for the coordinate system
        type_mask=0b0000111111111000,        # Bitmask to indicate which dimensions should be ignored (0b0000111111111000 means all ignored except position)
        lat_int=int(latitude * 1e7),       # Latitude in degrees * 1e7 (to convert to integer)
        lon_int=int(longitude * 1e7),      # Longitude in degrees * 1e7 (to convert to integer)
        alt=altitude,           # Altitude in meters (converted to millimeters)
        vx=0,                         # X velocity in m/s (not used)
        vy=0,                         # Y velocity in m/s (not used)
        vz=0,                         # Z velocity in m/s (not used)
        afx=0, afy=0, afz=0,                   # Accel x, y, z (not used)
        yaw=0, yaw_rate=0                       # Yaw and yaw rate (not used)
    )
    vehicle.mav.send(msg)


def get_heading(vehicle):

    vehicle.mav.command_long_send(vehicle.target_system,
    vehicle.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,74,0,0,0,0,0,0)
    ack_msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    #### COMMAND_ACK has a message id of 512.

    msg = vehicle.recv_match(type='VFR_HUD',blocking=True)

    heading = msg.heading
        #print(heading)
    return heading #degrees



def find_device_address():
    global rfd900x_address
    context = pyudev.Context()
    devices = context.list_devices(subsystem = 'tty')
    address = list()
    for device in devices:
        # print(device)
        if device.get('ID_VENDOR_ID') == '10c4' and device.get('ID_MODEL_ID') == 'ea60' and device.device_path.split('/')[-4] == '1-1.1:1.0':
            # print(device.device_node, device.get('ID_VEDOR'), device.get('ID_VEDOR'), device.get('ID_MODEL'), device.get('ID_SERIAL_SHORT'), device.device_path)
            rfd900x_address = device.device_node 


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
    while True:
        data = mavlink_data_queue.get()
        if data[0] == 3:
            current_pos = data[1]
            if current_pos:
                lat_lead = current_pos['lat']/1e7
                lon_lead = current_pos['lon']/1e7
                lead_yaw = (current_pos['hdg'])/100
                print(lead_yaw)
                ######### follower2 ######
                yaw1 = 270  #on the right
                dist1 = 20
                lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
                print(lat1, lon1)
                ##### send msg to follower2 drone #############

                goto_waypoint(follower2,lat1,lon1,10) #altitude 1
                print("formation command sent.")
                time.sleep(0.15)
                dist_to_target = distance_between(follower2,lat1,lon1)
                print(f"distance to formation: {round(dist_to_target,2)} m.")
                time.sleep(1)
                break
        
        else:
            print("no data retrieved from the leader.")


def distance_to_leader(vehicle,leader_lat,leader_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - leader_lat)
    dlon = radians(current_lon - leader_lon)
    a = sin(dlat / 2)**2 + cos(radians(leader_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters

def get_local_position(vehicle):
    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    100,1)
    msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    pos_x = msg.x # Degrees
    pos_y = msg.y  # Degrees
    pos_z = msg.z  # Meters
    return [pos_x,pos_y,pos_z]

def send_velocity_setpoint(vehicle, vx, vy, vz):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,        # type_mask (only vx, vy, vz, yaw_rate)
        0, 0, 0,                    # position (not used)
        vx, vy, vz,                 # velocity in m/s
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )

def angle_difference_with_heading(lat1, lon1, heading, lat2, lon2): # angle difference relative to origin lat, lon, heading and current lat, lon

    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Compute the difference in the longitudes
    delta_lon = lon2 - lon1

    # Calculate the bearing from the reference point to the second location
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
    bearing_to_point = math.atan2(x, y)

    # Convert bearing from radians to degrees
    bearing_to_point = math.degrees(bearing_to_point)

    # Normalize the bearing to a value between 0 and 360 degrees
    bearing_to_point = (bearing_to_point + 360) % 360

    # Calculate the angle difference relative to the reference heading
    angle_diff = (bearing_to_point - heading + 360) % 360

    return angle_diff

def distance_between(vehicle,leader_lat,leader_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - leader_lat)
    dlon = radians(current_lon - leader_lon)
    a = sin(dlat / 2)**2 + cos(radians(leader_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters


#command to proceed with arming from leader

# while True:
#     msg = 
#     if msg=='':
#         break
#     else:
#         print("command not received from the leader.")

baud_rate = 57600
source_system_id = 2

find_device_address()
print(rfd900x_address)
sender = send_rfd900x_mavlink_data(rfd900x_address, baud_rate, source_system_id)
drone_data = mavlink_receiver_rfd900x(rfd900x_address, baud_rate, source_system_id)
drone_data.start_receiving()

follower2 = connect('/dev/ttyACM0')
print("follower2 connected.")

# follower2
VehicleMode(follower2,"GUIDED")
print("follower2 in GUIDED mode")
time.sleep(1)

#arm
arm(follower2)
print("arming the follower2")
time.sleep(5)

#takeoff
drone_takeoff(follower2,10)
print("taking off follower2")
while True:
    altitude = abs(get_local_position(follower2)[2])
    print(f"Altitude: {altitude} m.")
    if altitude>9:
        print("Target altitude reached.")
        break

relativePosition = 'left'

###### main #######
formation_row()

while True:

    pos = mavlink_data_queue.get()
    if pos[0]==3:
        #fetch leader velocity in all direction
        lead_vx = pos[1]['vx']/100
        lead_vy = pos[1]['vy']/100
        lead_vz = pos[1]['vz']/100
        leader_lat = pos[1]['lat']*1e-7
        leader_lon = pos[1]['lon']*1e-7
        leader_yaw = pos[1]['hdg']/100
        yaw1 = 270 #yaw 270 for left

        follower_offset_y = 20 #offset in left
        follower_offset_x = 5
        send_velocity_setpoint(follower2,lead_vx,lead_vy,0) #sending only vx and vy
        print(f"leader vx: {lead_vx} m/s leader vy: {lead_vy} m/s leader vz: {lead_vz} m/s")

        lat1, lon1 = relative_pos(leader_lat,leader_lon, follower_offset_y, leader_yaw, yaw1) #coordinates in 20m left direction
        #print(f"Leader Coordinates: Latitude: {leader_lat} Longitude: {leader_lon}")
        distance_v = abs(distance_between(follower2,lat1,lon1)) #vertical distance to leader
        distance_h = abs(( (distance_between(follower2,leader_lat,leader_lon))**2 - (distance_v)**2 )**(1/2))  #horizontal distance to leader
        diff_dist_y = follower_offset_y - distance_h  
        diff_dist_x = distance_v - follower_offset_x

        if abs(diff_dist_y)>=1: #formation correction if difference is greater than 1m. 
            send_velocity_setpoint(follower2,0,diff_dist_y,0) #formation velocity-y command
            print("velocity vy command sent for formation.")

        # #command to correct pos_x using velocity_x
        # if abs(diff_dist_x)>=1:
        #     pos_f = get_global_position(follower2)
        #     lat_f = pos_f[0]
        #     lon_f = pos_f[1]
        #     ang = angle_difference_with_heading(leader_lat,leader_lon,leader_yaw,lat_f,lon_f)

        #     if (180<ang<270): #if angle is between 180 and 270deg then drone will go forward
        #         send_velocity_setpoint(follower2,diff_dist_x,0,0)
        #         print("velocity vx command sent for formation.")

        #     if (360>ang>270): #if angle is between 360 and 270deg then drone will go backward
        #         send_velocity_setpoint(follower2,-diff_dist_x,0,0)
        #         print("velocity vx command sent for formation.")
           
        print(f"Distance to Leader: horizontal: {round(distance_h,2)} m. vertical: {round(distance_v,2)}")

    else:
        print("no data received from leader.")

        


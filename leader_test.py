#!/usr/bin/env python3

from math import radians, cos, sin, sqrt, atan2
from pymavlink import mavutil
import time
import csv
import matplotlib.pyplot as plt

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

def get_pos_vel(vehicle):
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
    vx = msg.vx/100
    vy = msg.vy/100
    vz = msg.vz/100
    hdg = msg.hdg/100
    return [lat,lon,alt,vx,vy,vz,hdg] #velocity in m/s

leader = connect('/dev/ttyACM0')
print("Leader FCU connected...")
time.sleep(1)

# VehicleMode(leader,"GUIDED")
# print("Leader in GUIDED mode.")
# time.sleep(1)

# arm(leader)
# print("arming the leader")
# time.sleep(5)

# drone_takeoff(leader,10)
# print("taking off to 10m.")
# while True:
#     altitude = -get_local_position(leader)[2]
#     print(f"Altitude: {altitude} m.")
#     if altitude>9:
#         print("Target altitude reached.")
#         break

#leader parameters
counter = 0
target_lat = float(input("Enter Latitude: ")) #target latitude
target_lon = float(input("Enter Longitude: ")) #target longitude

data = []
start_time = time.time()
while True:
    current_time = current_time = time.time() - start_time  # Time elapsed since start
    vel = get_pos_vel(leader)
    vel_x = vel[3]
    vel_y = vel[4]
    data.append([current_time, vel_x, vel_y])
    print(f"current time: {current_time} seconds vel x: {vel_x} m/s  vel y: {vel_y} m/s")
    altitude = -get_local_position(leader)[2]
    if altitude<=5:
        break

    # counter+=1
    # if counter==1:
    #     goto_waypoint(leader,target_lat,target_lon,10)
    #     print("Heading towards target waypoint.")
    #     time.sleep(1)

    # distance = distance_to_target(leader,target_lat,target_lon)
    # print(f"Distance to Target Waypoint: {round(distance,2)} m.")
    # if distance<=2:
    #     print("Target Reached.")
    #     break


end_time = time.time()
total_time = abs(int(end_time-start_time))
print(total_time,"seconds")

# Save data to a CSV file
csv_file = r'/home/pi/Desktop/velocity_data.csv'
with open(csv_file, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Velocity X (m/s)", "Velocity Y (m/s)"])
    writer.writerows(data)

print(f"Data saved to {csv_file}")

# Plotting Velocity X and Y against Time
times = [row[0] for row in data]
velocities_x = [row[1] for row in data]
velocities_y = [row[2] for row in data]

plt.figure()
plt.plot(times, velocities_x, label='Velocity X')
plt.plot(times, velocities_y, label='Velocity Y')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity X and Y over Time')
plt.legend()
plt.grid(True)
plt.savefig(r'/home/pi/Desktop/velocity_plot.png')  # Save the plot as an image file
#plt.show()



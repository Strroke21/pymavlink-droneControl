import math
from dronekit import connect
# Constants

# Connect to the vehicle
connection_string = 'tcp:127.0.0.1:5763'  # Replace with your connection string
vehicle = connect(connection_string, wait_ready=True)

# Get the current heading of the leader drone
#current_heading = vehicle.heading  # Heading in degrees
#print(current_heading)
EARTH_RADIUS = 6371000  # Radius of Earth in meters

def calculate_new_position(lat, lon, heading, follower_heading, distance):

    # Convert heading to radians and calculate new heading
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

# Example usage
leader_lat = 19.1340793  # Leader drone's latitude (in degrees)
leader_lon = 72.912634  # Leader drone's longitude (in degrees)
leader_heading = 280  # Leader drone's heading (in degrees)
distance_to_follower_1 = 30  # Distance to follower drone (in meters)
follower_1_heading = 90
follower_1_lat, follower_1_lon = calculate_new_position(
    leader_lat, leader_lon, leader_heading, follower_1_heading, distance_to_follower_1)

print(f"Follower drone_1's latitude: {follower_1_lat}")
print(f"Follower drone_1's longitude: {follower_1_lon}")

distance_to_follower_2 = 30 / math.cos(math.radians(45))  # Distance to follower drone (in meters)
follower_2_heading = 135
follower_2_lat, follower_2_lon = calculate_new_position(
    leader_lat, leader_lon, leader_heading, follower_2_heading, distance_to_follower_2)

print(f"Follower drone_2's latitude: {follower_2_lat}")
print(f"Follower drone_2's longitude: {follower_2_lon}")

distance_to_follower_3 = 30 / math.cos(math.radians(45))  # Distance to follower drone (in meters)
follower_3_heading = 225
follower_3_lat, follower_3_lon = calculate_new_position(
    leader_lat, leader_lon, leader_heading, follower_3_heading, distance_to_follower_3)

print(f"Follower drone_3's latitude: {follower_3_lat}")
print(f"Follower drone_3's longitude: {follower_3_lon}")

distance_to_follower_4 = 30  # Distance to follower drone (in meters)
follower_4_heading = 270
follower_4_lat, follower_4_lon = calculate_new_position(
    leader_lat, leader_lon, leader_heading, follower_4_heading, distance_to_follower_4)

print(f"Follower drone_4's latitude: {follower_4_lat}")
print(f"Follower drone_4's longitude: {follower_4_lon}")
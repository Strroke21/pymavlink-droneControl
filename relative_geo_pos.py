import math

def calculate_new_coordinates(latitude, longitude, distance_east):

    # Earth's radius at the equator in meters
    R = 6378137
    # Convert latitude to radians
    lat_rad = math.radians(latitude)
    # One degree of longitude in meters at the given latitude
    one_deg_longitude = R * math.cos(lat_rad) * (math.pi / 180)
    # Convert distance to degrees
    longitude_degrees = distance_east / one_deg_longitude
    # Calculate new longitude
    new_longitude = longitude + longitude_degrees
    # Latitude remains the same
    new_latitude = latitude
    return new_latitude, new_longitude



def calculate_new_coordinates_west(latitude, longitude, distance_west):

    # Earth's radius at the equator in meters
    R = 6378137
    # Convert latitude to radians
    lat_rad = math.radians(latitude)
    # One degree of longitude in meters at the given latitude
    one_deg_longitude = R * math.cos(lat_rad) * (math.pi / 180)
    # Convert distance to degrees
    longitude_degrees = distance_west / one_deg_longitude
    # Calculate new longitude
    new_longitude = longitude - longitude_degrees
    # Latitude remains the same
    new_latitude = latitude
    return new_latitude, new_longitude


# Example usage
latitude = 19.1343888
longitude = 72.9125637
distance_east = 20  # Distance in meters

new_latitude, new_longitude = calculate_new_coordinates(latitude, longitude, distance_east)

print(f"New coordinates: Latitude = {new_latitude}, Longitude = {new_longitude}")

# Example usage
latitude = 19.1343888
longitude = 72.9125637
distance_west = 20  # Distance in meters

new_latitude, new_longitude = calculate_new_coordinates_west(latitude, longitude, distance_west)

print(f"New coordinates: Latitude = {new_latitude}, Longitude = {new_longitude}")
print(math.pi)

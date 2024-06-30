import math

def calculate_destination(lat, lon, distance, heading):
    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    
    # Convert heading to radians
    theta = math.radians(heading)
    
    # Radius of the Earth
    R = 6371e3  # in meters
    
    # Calculate the destination latitude
    lat2 = math.asin(math.sin(lat1) * math.cos(distance / R) +
                     math.cos(lat1) * math.sin(distance / R) * math.cos(theta))
    
    # Calculate the destination longitude
    lon2 = lon1 + math.atan2(math.sin(theta) * math.sin(distance / R) * math.cos(lat1),
                             math.cos(distance / R) - math.sin(lat1) * math.sin(lat2))
    
    # Convert destination coordinates from radians to degrees
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    
    return lat2, lon2

# Example usage:
current_lat = 37.7749   # Latitude of the current location
current_lon = -122.4194 # Longitude of the current location
distance = 1000         # Distance to the new point in meters
heading = 75            # Heading in degrees

new_lat, new_lon = calculate_destination(current_lat, current_lon, distance, heading)
print(f"New coordinates: Latitude = {new_lat}, Longitude = {new_lon}")


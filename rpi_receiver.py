
####rpi_receiver######
from droneControl import connect, goto_waypoint, drone_takeoff, VehicleMode, arm
import time

vehicle = connect('/dev/ttyACM0')
print("vehicle connected...")
counter = 0

while True:

    ########## RFD receiver #######
    data = 
    lat = 
    lon = 
    ###############################
    if data:
        counter += 1
        if counter == 1:
            VehicleMode(vehicle,"GUIDED")
            time.sleep(0.2)
            arm(vehicle)
            time.sleep(0.2)
            drone_takeoff(vehicle,10)
            time.sleep(10)

        goto_waypoint(vehicle, lat, lon, 0)  #altitude = 0 to use current altitude
        print(f"Target waypoint sent. Latitude: {lat} Longitude: {lon}")
        time.sleep(2)

    else:
        print("No data received from the Leader")



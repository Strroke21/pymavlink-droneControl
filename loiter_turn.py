from pymavlink import mavutil

# Connect to the drone
master = mavutil.mavlink_connection('tcp:127.0.0.1:5763')

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()

def send_loiter_turn_command(master, target_system, target_component, number_of_turns, radius, latitude, longitude, altitude):
    master.mav.command_long_send(
        target_system,       # target_system
        target_component,    # target_component
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,  # command
        0,                   # confirmation
        number_of_turns,     # param1: Number of turns
        0,                   # param2: Empty
        0,                   # param3: Empty
        radius,              # param4: Radius (positive for clockwise, negative for counterclockwise)
        latitude,            # param5: Latitude
        longitude,           # param6: Longitude
        altitude             # param7: Altitude
    )

# Example parameters
target_system = master.target_system
target_component = master.target_component
number_of_turns = 1
radius = 30  # meters
latitude = -35.3638100 # example latitude in degrees * 1E7
longitude = 149.1648719  # example longitude in degrees * 1E7
altitude = 10 # meters

# Send the loiter turn command
send_loiter_turn_command(master, target_system, target_component, number_of_turns, radius, latitude, longitude, altitude)

while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    if msg.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS:
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Loiter turn command accepted")
        else:
            print("Loiter turn command failed")
        break





# pymavlink-droneControl
# droneControl.py

This script provides various functions to control a drone using the MAVLink protocol with the `pymavlink` library. 

## Prerequisites

- Python 3.x
- `pymavlink` library

You can install `pymavlink` using pip:

```
pip install pymavlink
```
# 1. Connecting to the Drone
```bash
from droneControl import connect

connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string)
```
# 2. Changing the Vehicle Mode
```
from droneControl import VehicleMode

# Example: Changing to GUIDED mode
VehicleMode(vehicle,"LOITER")
#mode:  ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE", "LAND"]
```
# 3. Arming the Drone
```
from droneControl import arm

arm(vehicle)
```
# 4. Takeoff
```
from droneControl import drone_takeoff

# Example: Taking off to an altitude of 10 meters
drone_takeoff(vehicle, 10)
```
# 5. Sending velocity setpoints
```
from droneControl import send_velocity_setpoint

# Example: Moving the drone at a velocity of 1 m/s in the x-direction
send_velocity_setpoint(vehicle, 1, 0, 0)
```
# 6. Getting local position
```
from droneControl import get_local_position

position = get_local_position(vehicle)
print(f"Local Position: {position}")
```
# 7. go to a Waypoint
```
from droneControl import goto_waypoint

# Example: Flying to a waypoint at latitude 37.7749, longitude -122.4194, altitude 20 meters
goto_waypoint(vehicle, 37.7749, -122.4194, 20)
```
# 8. Getting global position
```
from droneControl import get_global_position

global_position = get_global_position(vehicle)
print(f"Global Position: {global_position}")
```
# 9. Sending position setpoints
```
from droneControl import send_position_setpoint

# Example: Moving the drone to a position offset of 5 meters in the x-direction, 5 meters in the y-direction, and 5 meters in the z-direction
send_position_setpoint(vehicle, 5, 5, 5)
```
# 10. Setting Yaw
```
from droneControl import condition_yaw

# Example: Setting yaw to 90 degrees
condition_yaw(vehicle, 90,1)
```


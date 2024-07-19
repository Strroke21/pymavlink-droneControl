# pymavlink-droneControl
# droneControl.py

This script provides various functions to control a drone using the MAVLink protocol with the `pymavlink` library. 

## Prerequisites

- Python 3.x
- `pymavlink` library

You can install `pymavlink` using pip:

```bash
pip install pymavlink
```
# 1. Connecting to the Drone
```bash
from droneControl import connect

connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string)
```
# 2. Changing the Vehicle Mode
```bash
from droneControl import VehicleMode

# Example: Changing to GUIDED mode
VehicleMode(4, vehicle)
#mode_id = 0:STABILIZE, 1:ACRO, 2: ALT_HOLD, 3:AUTO, 4:GUIDED, 5:LOITER, 6:RTL, 7:CIRCLE, 9:LAND
```

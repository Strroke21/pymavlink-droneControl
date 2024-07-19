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

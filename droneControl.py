#!/usr/bin/env python3

from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2


def connect(connection_string,baud):

    vehicle =  mavutil.mavlink_connection(connection_string,baud)

    return vehicle

def VehicleMode(vehicle,mode):

    modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE","", "LAND"]
    if mode in modes:
        mode_id = modes.index(mode)
    else:
        mode_id = 12
    ##### changing to guided mode #####
    #mode_id = 0:STABILIZE, 1:ACRO, 2: ALT_HOLD, 3:AUTO, 4:GUIDED, 5:LOITER, 6:RTL, 7:CIRCLE, 9:LAND 12:None
    vehicle.mav.set_mode_send(
        vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
    

def flightMode(vehicle):
    
    # Wait for a 'HEARTBEAT' message
    mode = vehicle.flightmode

    return mode

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
    	                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
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


def send_velocity_setpoint(vehicle, vx, vy, vz):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,        # type_mask (only vx, vy, vz, yaw_rate)
        0, 0, 0,                    # position (not used)
        vx, vy, vz,                 # velocity in m/s
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )
 
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

#target waypoint function

def goto_waypoint(vehicle,latitude, longitude, altitude):
    msg = vehicle.mav.set_position_target_global_int_encode(
        time_boot_ms=10,
        target_system=vehicle.target_system,       # Target system (usually 1 for drones)
        target_component=vehicle.target_component,    # Target component (usually 1 for drones)
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame of reference for the coordinate system
        type_mask=0b0000111111111000,        # Bitmask to indicate which dimensions should be ignored (0b0000111111111000 means all ignored except position)
        lat_int=int(latitude * 1e7),       # Latitude in degrees * 1e7 (to convert to integer)
        lon_int=int(longitude * 1e7),      # Longitude in degrees * 1e7 (to convert to integer)
        alt=altitude,
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
    vx = msg.vx/100 #in m/s
    vy= msg.vy/100 #in m/s
    vz = msg.vz/100 #in m/s
    local_alt = msg.relative_alt/100
    return [lat,lon,alt,vx,vy,vz,local_alt]

def send_position_setpoint(vehicle, pos_x, pos_y, pos_z):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b110111111000,        # type_mask (only for postion)
        pos_x, pos_y, pos_z,   # position 
        0, 0, 0,                 # velocity in m/s (not used)
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )


############## desired yaw function ##############  
def condition_yaw(vehicle, yaw, relative):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg=vehicle.mav.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        yaw,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw, 0 for short turn
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
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


def get_battery_status(vehicle):
    # Request the SYS_STATUS message
    vehicle.mav.request_data_stream_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        1, 1  # Requesting one message per second
    )

    # Wait for a 'SYS_STATUS' message
    message = vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=10)

    if message is None:
        raise TimeoutError("Did not receive SYS_STATUS message in time")

    battery_voltage = message.voltage_battery / 1000.0  # Convert mV to V
    battery_current = message.current_battery / 100.0  # Convert cA to A
    battery_remaining = message.battery_remaining  # Remaining battery percentage

    return {
        'voltage': battery_voltage,
        'current': battery_current,
        'remaining %': battery_remaining
    }

def get_system_status(vehicle):
    # Wait for a 'HEARTBEAT' message
    message = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

    if message is None:
        raise TimeoutError("Did not receive HEARTBEAT message in time")

    # Interpret the system status
    system_status = message.system_status

    status_mapping = {
        mavutil.mavlink.MAV_STATE_UNINIT: 'Uninitialized',
        mavutil.mavlink.MAV_STATE_BOOT: 'Booting',
        mavutil.mavlink.MAV_STATE_CALIBRATING: 'Calibrating',
        mavutil.mavlink.MAV_STATE_STANDBY: 'Standby',
        mavutil.mavlink.MAV_STATE_ACTIVE: 'Active',
        mavutil.mavlink.MAV_STATE_CRITICAL: 'Critical',
        mavutil.mavlink.MAV_STATE_EMERGENCY: 'Emergency',
        mavutil.mavlink.MAV_STATE_POWEROFF: 'Powered Off',
        mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION: 'Flight Termination',
    }

    return status_mapping.get(system_status, 'Unknown')


def home_location(vehicle):

    vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,0,0,0,0,0,0,0,0)
    msg=vehicle.recv_match(type='HOME_POSITION',blocking=True)
    if msg:
        return [msg.latitude * 1e-7, msg.longitude * 1e-7,msg.altitude * 1e-3]
    
    #0:lat 1:lon 2:alt


def distance_to_home(vehicle):

    msg1=home_location(vehicle)
    
    home_lat=msg1[0]
    home_lon=msg1[1]
    #home_alt=msg1.altitude * 1e-3

    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - home_lat)
    dlon = radians(current_lon - home_lon)
    a = sin(dlat / 2)**2 + cos(radians(home_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters


def scaled_imu_data(vehicle):

    vehicle.wait_heartbeat()
    msg = vehicle.recv_match(type='SCALED_IMU2', blocking=True)
    #print(msg)
    if msg is not None:
        time_boot_ms = msg.time_boot_ms
        accel_x = msg.xacc
        accel_y = msg.yacc
        accel_z = msg.zacc
        gyro_x = msg.xgyro
        gyro_y = msg.ygyro
        gyro_z = msg.zgyro
        mag_x = msg.xmag
        mag_y = msg.ymag
        mag_z = msg.zmag

        return {'x_accel':accel_x, 'y_accel':accel_y, 'z_accel':accel_z, 'x_gyro':gyro_x, 'y_gyro':gyro_y, 'z_gyro':gyro_z, 'x_mag':mag_x,'y_mag':mag_y, 'z_mag':mag_z}
    
    #accel_unit: mG gyro_unit: mrad/s mag_unit:mgauss

def send_land_message(vehicle,x,y):
    msg = vehicle.mav.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.mav.send(msg)

def arm_status(vehicle):
    heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if heartbeat:
        armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            return True
        else:
            return False
    
def set_parameter(vehicle, param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(
        vehicle.target_system,     # Target system ID (usually 1)
        vehicle.target_component,  # Target component ID (usually 1)
        param_id.encode('utf-8'),     # Parameter ID as bytes
        param_value,                  # New parameter value
        param_type                    # MAVLink type (e.g., float, int)
    )

def get_rangefinder_data(vehicle):
    # Wait for a DISTANCE_SENSOR or RANGEFINDER message
    msg = vehicle.recv_match(type=['DISTANCE_SENSOR', 'RANGEFINDER'], blocking=True)
    if msg:
        if msg.get_type() == 'DISTANCE_SENSOR':
            distance = msg.current_distance/100  # in meters
        elif msg.get_type() == 'RANGEFINDER':
            distance = msg.distance  # in meters
        return distance
    else:
        return None


    





    
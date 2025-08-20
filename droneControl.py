#!/usr/bin/env python3 

from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2
import time
import os

os.environ["MAVLINK20"] = "1"

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

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
    while True:
        vehicle.recv_match(type='HEARTBEAT', blocking=True)
        # Wait for a 'HEARTBEAT' message
        mode = vehicle.flightmode

        return mode

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
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
    while True:
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is not None:
            pos_x = msg.x # meters
            pos_y = msg.y  # meters
            pos_z = msg.z  # Meters
            vx = msg.vx
            vy = msg.vy
            vz = msg.vz
            return [pos_x,pos_y,pos_z,vx,vy,vz]

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
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            lat = msg.lat/1e7 # lat
            lon = msg.lon/1e7 # lon
            alt = msg.alt/1000  # alt
            vx = msg.vx/100 #in m/s
            vy= msg.vy/100 #in m/s
            vz = msg.vz/100 #in m/s
            relative_alt = msg.relative_alt/100 #in m
            hdg = msg.hdg/100 #in deg
            time_boot_ms = msg.time_boot_ms
            return [lat,lon,alt,vx,vy,vz,relative_alt,hdg, time_boot_ms]

def send_position_setpoint(vehicle, pos_x, pos_y, pos_z):

    # Send MAVLink command to set position
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
    #### COMMAND_ACK has a message id of 512.

    msg = vehicle.recv_match(type='VFR_HUD',blocking=True)

    heading = msg.heading
        #print(heading)
    return heading #degrees


def get_battery_status(vehicle):
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
    while True:
        msg=vehicle.recv_match(type='HOME_POSITION',blocking=True)
        if msg is not None:
            return [msg.latitude * 1e-7, msg.longitude * 1e-7,msg.altitude * 1e-3]
        

def distance_to_home(vehicle):

    while True:
        msg1=home_location(vehicle)
        if msg1 is not None:
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
    while True:
        msg = vehicle.recv_match(type='SCALED_IMU', blocking=True)
        if msg is not None:
            x_accel = msg.xacc
            y_accel = msg.yacc
            z_accel = msg.zacc
            x_gyro = msg.xgyro
            y_gyro = msg.ygyro
            z_gyro = msg.zgyro

            return [x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro]



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
        armed = vehicle.motors_armed()
        if armed==128:
            return True
        else:
            return False
    
def set_parameter(vehicle, param_name, param_value):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(
        vehicle.target_system,
        vehicle.target_component,
        param_name.encode('utf-8'),
        param_value,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    #usage set_parameter(vehicle, "PARAM_NAME", 1)

def get_rangefinder_data(vehicle):
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            distance = msg.current_distance/100  # in meters
            return distance


def arm_and_takeoff(vehicle,target_alt):

    while True:
        VehicleMode(vehicle,"GUIDED")
        print("vehicle in GUIDED mode")
        time.sleep(1)
        arm(vehicle)
        time.sleep(1)
        drone_takeoff(vehicle,target_alt)
        ack_msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"{ack_msg.result}: Takeoff Successful...")
                break

        else:
            
            print("Arm and Takeoff Failed...")
            time.sleep(1)

    while True:
        altitude = abs(get_local_position(vehicle)[2])
        print(f"Altitude: {altitude:.2f} m.")
        if altitude>(target_alt*0.8):
            print("Target altitude reached.")
            time.sleep(0.1)
            break

def attitude(vehicle):
    while True:
        msg = vehicle.recv_match(type="ATTITUDE", blocking=True)
        if msg is not None:
            return msg

    

def rc_channels(vehicle):
    while True:
        msg = vehicle.recv_match(type="RC_CHANNELS", blocking=True)
        if msg is not None:
            return msg


def vision_position_send(vehicle, x, y, z, roll, pitch, yaw):

    msg = vehicle.mav.vision_position_estimate_encode(
        int(time.time() * 1e6),
        x, y, z,
        roll, pitch, yaw  
    )
    vehicle.mav.send(msg)

def vision_speed_send(vehicle, vx, vy, vz):

    msg = vehicle.mav.vision_speed_estimate_encode(
        int(time.time() * 1e6),
        vx, vy, vz
        )
    vehicle.mav.send(msg)


def set_default_global_origin(vehicle, home_lat, home_lon, home_alt):
    vehicle.mav.set_gps_global_origin_send(
        1,
        home_lat, 
        home_lon,
        home_alt
    )

def set_default_home_position(vehicle, home_lat, home_lon, home_alt):
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    vehicle.mav.set_home_position_send(
        1,
        int(home_lat * 1e7), 
        int(home_lon * 1e7),
        int(home_alt),
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )

def send_gps_input(vehicle, lat, lon, alt, hdop,vdop, vx, vy, vz, sats):

    vehicle.mav.gps_input_send(
        int(time.time() * 1e6),  # time_usec (microseconds)
        0,                       # gps_id
        0,                       # flags
        0,           # ignore_flags
        0,                     # time_week_ms (GPS time in ms, set 0 if not synced)
        3,                      # fix_type (3 = 3D fix)
        int(lat*1e7),               # lat (degrees * 1E7)
        int(lon*1e7),               # lon (degrees * 1E7)
        int(alt*1e7),               # alt (millimeters)
        hdop, vdop,                   # hdop, vdop
        float(vx), float(vy), float(-vz),  # velocity in m/s (z is down in NED)
        0,                    # speed_accuracy (m/s)
        0,                    # horiz_accuracy (m)
        0,                    # vert_accuracy (m)
        sats                      # satellites_visible (optional)
    )

def gps_status(vehicle):

    while True:
        msg = vehicle.recv_match(type='GPS_STATUS', blocking=True)
        if msg:
            sats = msg.satellites_visible
            satellite_ids = msg.satellite_prn
            satellites_used = msg.satellite_used #bool 0:not used, 1:used
            sat_elevation = msg.satellite_elevation #degrees
            sat_azimuth = msg.satellite_azimuth #degrees (direction of the satellite)
            sat_snr = msg.satellite_snr #signal to noise ratio dB
            return [sats, satellite_ids, satellites_used, sat_elevation, sat_azimuth, sat_snr]
        
def attitude_quaternion_cov(vehicle):
    while True:
        msg = vehicle.recv_match(type='ATTITUDE_QUATERNION_COV', blocking=True)
        if msg:
            q = msg.q #Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            roll_speed = msg.rollspeed # Roll angular speed in rad/s
            pitch_speed = msg.pitchspeed # Pitch angular speed in rad/s
            yaw_speed = msg.yawspeed # Yaw angular speed in rad/s
            covariance = msg.covariance # 6x6 covariance matrix for attitude
            return [q, roll_speed, pitch_speed, yaw_speed, covariance]
        
def nav_controller_output(vehicle):
    while True:
        msg = vehicle.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        if msg:
            nav_roll = msg.nav_roll # current Desired roll angle in degrees
            nav_pitch = msg.nav_pitch # current Desired pitch angle in degrees
            nav_bearing = msg.nav_bearing # current Desired bearing in degrees
            target_bearing = msg.target_bearing # Target bearing in degrees
            wp_dist = msg.wp_dist # Distance to the active waypoint in meters
            alt_error = msg.alt_error # Altitude error in meters
            aspd_error = msg.aspd_error # Airspeed error in m/s
            xtrack_error = msg.xtrack_error # Cross-track error in x-y plane in meters
            return [nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error]
        

def set_attitude_target(vehicle, q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust):
    
    msg = vehicle.mav.set_attitude_target_encode(
        int(time.time() * 1e6),  # time_boot_ms
        vehicle.target_system,  # target_system
        vehicle.target_component,  # target_component
        0b0000000,  # type_mask (0 means all fields are valid)
        q,  # quaternion (w, x, y, z) or [1, 0, 0, 0] for no rotation
        body_roll_rate,  # body roll rate in rad/s
        body_pitch_rate,  # body pitch rate in rad/s
        body_yaw_rate,  # body yaw rate in rad/s
        thrust  # thrust (0.0 to 1.0, where 1.0 is full thrust) (-1 to 1) for thrust reverse capable systems
    )
    vehicle.mav.send(msg)

def optical_flow(vehicle, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance):
    msg = vehicle.mav.set_optical_flow_encode(
        int(time.time() * 1e6),  # time_usec
        0,  # sensor_id (0 for default sensor)
        flow_x,  # flow_x (flow in x direction in pixels)
        flow_y,  # flow_y (flow in y direction in pixels)
        flow_comp_m_x,  # flow_comp_m_x (compensated flow in x direction in m/s)
        flow_comp_m_y,  # flow_comp_m_y (compensated flow in y direction in m/s)
        quality,  # quality (0 to 255, where 255 is best quality)
        ground_distance  # ground_distance (positive: known, negative: unknown )
    )
    vehicle.mav.send(msg)

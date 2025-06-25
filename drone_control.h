// drone_utils.h
#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavlink/common/mavlink.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip> 
#include <cmath>
#include <atomic>  // For thread-safe flag
#include <vector>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <map>

using namespace mavsdk;

// Function to calculate 2D distance
double distance_between(double current_lat, double current_lon, double leader_lat, double leader_lon);

// Function to calculate distance to home
double distance_to_home(System& system);

//go to target location
bool goto_waypoint(System& system, float target_lat, float target_lon, float target_alt);

//set velocity
void set_velocity(System& system, float vx, float vy, float vz);

//takeoff
bool takeoff(System& system, float takeoff_altitude_m);

//set flight mode
bool set_flight_mode(System& system, uint8_t base_mode, uint8_t custom_mode);

//get current position
std::pair<double, double> get_global_position(System& system);

//get current heading
float current_heading(System& system);

//send vision speed
void vision_speed_send(System& system, float vx, float vy, float vz);

//send vision position
void vision_position_send(System& system, float x, float y, float z, float roll, float pitch, float yaw);

//set gps global origin
void set_default_global_origin(System& system, int32_t home_lat, int32_t home_lon, int32_t home_alt);

//set home origin
void set_default_home_position(System& system, int32_t home_lat, int32_t home_lon, int32_t home_alt);

#endif // DRONE_UTILS_H

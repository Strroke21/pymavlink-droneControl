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
double distance_to_home(mavsdk::System& system, mavsdk::Telemetry& telemetry);

#endif // DRONE_UTILS_H

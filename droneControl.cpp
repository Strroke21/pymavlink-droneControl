#include <drone_control.h>


//2d distance calculation
double distance_between(double current_lat, double current_lon, double leader_lat, double leader_lon) {
    // Convert degrees to radians
    const double R = 6378137.0;  // Earth radius in meters
    double dlat = (current_lat - leader_lat) * M_PI / 180.0;
    double dlon = (current_lon - leader_lon) * M_PI / 180.0;

    // Haversine formula
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(leader_lat * M_PI / 180.0) * std::cos(current_lat * M_PI / 180.0) * 
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // Distance in meters
    double distance = R * c;

    return distance;
}

double distance_to_home(System& system) {
    Telemetry telemetry{system}; //initialize telemetry
    // Get current position (non-blocking snapshot)
    Telemetry::Position position = telemetry.position();
    double current_lat = position.latitude_deg;
    double current_lon  = position.longitude_deg;
    std::cout << std::fixed << std::setprecision(7);
    std::cout << "Current position: " << current_lat << ", " << current_lon << std::endl;

    // Get home position
    Telemetry::Position home = telemetry.home();
    double home_lat = home.latitude_deg;
    double home_lon  = home.longitude_deg;

    return distance_between(current_lat, current_lon, home_lat, home_lon);
}

bool goto_waypoint(System& system, float target_lat, float target_lon, float target_alt)
{
    MavlinkPassthrough mavlink_passthrough{system};
    Telemetry telemetry{system};

    mavlink_message_t msg; // NOT const here

    mavlink_msg_set_position_target_global_int_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        static_cast<uint32_t>(0),
        1, // target system
        1, // target component
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        static_cast<int32_t>(target_lat * 1e7),
        static_cast<int32_t>(target_lon * 1e7),
        target_alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    );

    // Safe to send as const now
    auto result = mavlink_passthrough.send_message(msg);

    if (result != MavlinkPassthrough::Result::Success) {
        std::cerr << "Failed to send waypoint: " << static_cast<int>(result) << std::endl;
        return false;
    }
    
    std::cout << "Goto command sent to reach target waypoint." << std::endl;

    bool target_reached = false;
    const float tolerance = 2.0f; // meters

    while (!target_reached) {
        Telemetry::Position position = telemetry.position();

        double current_lat = position.latitude_deg;
        double current_lon = position.longitude_deg;

        double dist = distance_between(current_lat, current_lon, target_lat, target_lon);
        std::cout << "Distance to target: " << dist << " meters." << std::endl;

        if (dist <= tolerance) {
            std::cout << "Target reached!" << std::endl;
            target_reached = true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return true;

}

void set_velocity(System& system, float vx, float vy, float vz) {
    MavlinkPassthrough mavlink_passthrough{system};

    mavlink_message_t msg;

    mavlink_msg_set_position_target_local_ned_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        static_cast<uint32_t>(0),   // time_boot_ms (can be 0 or from telemetry)
        1,                          // target_system (usually 1)
        1,                          // target_component (usually 1)
        MAV_FRAME_LOCAL_NED,       // coordinate frame
        0b0000111111000111,         // type_mask (only vx, vy, vz are active)
        0.0f, 0.0f, 0.0f,           // x, y, z position (ignored)
        vx, vy, vz,                 // velocity in m/s
        0.0f, 0.0f, 0.0f,           // acceleration (ignored)
        0.0f, 0.0f                  // yaw, yaw_rate (ignored)
    );

    mavlink_passthrough.send_message(msg);
}

bool takeoff(System& system, float takeoff_altitude_m) {
    Action action(system);
    Telemetry telemetry(system);

    // Set desired takeoff altitude
    auto altitude_result = action.set_takeoff_altitude(takeoff_altitude_m);
    if (altitude_result != Action::Result::Success) {
        std::cerr << "Failed to set takeoff altitude: " << altitude_result << std::endl;
        return false;
    }

    // Wait for system to be ready
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Arm the drone
    auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return false;
    }

    std::cout << "Armed successfully." << std::endl;

    // Takeoff
    auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        return false;
    }

    std::cout << "Takeoff initiated to " << takeoff_altitude_m << " meters..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return true;
    
}

bool set_flight_mode(System& system, uint8_t base_mode, uint8_t custom_mode) {
    MavlinkPassthrough mavlink_passthrough(system);

    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        system.get_system_id(),
        base_mode,
        custom_mode
    );

    return mavlink_passthrough.send_message(msg) == MavlinkPassthrough::Result::Success;
}

//get global position

std::pair<double, double> get_global_position(System& system) {
    Telemetry telemetry{system}; //initialize telemetry
    // Get current position (non-blocking snapshot)
    Telemetry::Position position = telemetry.position();
    double current_lat = position.latitude_deg;
    double current_lon  = position.longitude_deg;

    return std::make_pair(current_lat, current_lon);
}

//get current heading
float current_heading(System& system){
    Telemetry telemetry{system}; //initialize telemetry
    // Get current heading (non-blocking snapshot)
    Telemetry::EulerAngle euler_angle = telemetry.attitude_euler();
    float current_heading = euler_angle.yaw_deg;

    if (current_heading < 0) {
        current_heading += 360.0f; // Normalize to [0, 360)
    }

}
//g++ -c droneControl.cpp -lmavsdk -lpthread -o droneControl (-c argument to create object file)
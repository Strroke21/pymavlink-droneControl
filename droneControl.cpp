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

double distance_to_home(System& system, Telemetry& telemetry) {
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


//g++ -c droneControl.cpp -lmavsdk -lpthread -o droneControl (-c argument to create object file)
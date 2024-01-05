//
// Created by Charles (Walrus) on 5/01/2024.
//

#include <iostream>

class GNSS {
public:
    double latitude;
    double longitude;
    double altitude;

    GNSS();

    // simulate GNSS update based on time
    void simulate(double time);
};

GNSS::GNSS() : latitude(0.0), longitude(0.0), altitude(0.0) {}

void GNSS::simulate(double time) {
    // simple simulation: GNSS position changes linearly with time
    latitude = 0.1 * time;
    longitude = 0.2 * time;
    altitude = 10000.0 + 500.0 * time;

    std::cout << "Simulated GNSS Data:\n"
              << "Latitude: " << latitude << " degrees\n"
              << "Longitude: " << longitude << " degrees\n"
              << "Altitude: " << altitude << " meters\n\n";
}

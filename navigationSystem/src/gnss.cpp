//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "gnss.hpp"

GNSS::GNSS() : latitude(0.0), longitude(0.0), altitude(0.0), speedX(0.0),
                speedY(0.0), heading(0.0) {}

void GNSS::setGNSS(double inputLatitude, double inputLongitude, double inputAltitude, double inputSpeedX,
                   double inputSpeedY, double inputHeading) {
    this->latitude = inputLatitude;
    this->longitude = inputLongitude;
    this->altitude = inputAltitude;
    this->speedX = inputSpeedX;
    this->speedY = inputSpeedY;
    this->heading = inputHeading;
}

double GNSS::getGNSSlatitude() const {
    return latitude;
}

double GNSS::getGNSSlongitude() const {
    return longitude;
}

double GNSS::getGNSSalt() const {
    return altitude;
}

double GNSS::getGNSSspeedX() const {
    return speedX;
}

double GNSS::getGNSSspeedY() const {
    return speedY;
}

double GNSS::getGNSSheading() const {
    return heading;
}

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







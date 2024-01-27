//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "gnss.hpp"

GNSS::GNSS(double &newLatitude, double &newLongitude, double &newAltitude, double &newSpeedX, double &newSpeedY,
           double &newHeading, double &measurementError)
           : latitude(newLatitude), longitude(newLongitude), altitude(newAltitude), speedX(newSpeedX), speedY(newSpeedY),
             heading(newHeading), measurementError(measurementError) {}

void GNSS::setGNSS(const double inputLatitude, const double inputLongitude, const double inputAltitude,
                   const double inputSpeedX, const double inputSpeedY, const double inputHeading) {
    this->latitude = inputLatitude;
    this->longitude = inputLongitude;
    this->altitude = inputAltitude;
    this->speedX = inputSpeedX;
    this->speedY = inputSpeedY;
    this->heading = inputHeading;
}

void GNSS::updateGNSSlatitude(const double newLat) {
    this->latitude = newLat;
}

double GNSS::getGNSSlatitude() const {
    return this->latitude;
}

void GNSS::updateGNSSlongitude(const double newLong) {
    this->longitude = newLong;
}

double GNSS::getGNSSlongitude() const {
    return longitude;
}

void GNSS::updateGNSSalt(const double newAlt) {
    this->altitude = newAlt;
}

double GNSS::getGNSSalt() const {
    return this->altitude;
}

void GNSS::updateGNSSspeedX(const double newSpeed) {
    this->speedX = newSpeed;
}

double GNSS::getGNSSspeedX() const {
    return speedX;
}

void GNSS::updateGNSSspeedY(const double newSpeed) {
    this->speedY = newSpeed;
}

double GNSS::getGNSSspeedY() const {
    return speedY;
}

void GNSS::updateGNSSHeading(double newHeading) {
    this->heading = newHeading;
}

double GNSS::getGNSSheading() const {
    return heading;
}

double &GNSS::updateMeasurementError() {
    return measurementError = 2;    // arbitrary value pending complete implementation
}




















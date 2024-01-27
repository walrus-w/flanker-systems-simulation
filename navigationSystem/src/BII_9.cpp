//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "BII_9.hpp"

// constructor for cold and dark aircraft
BII_9::BII_9(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
             RingLaserGyroscope& rlgRollArg, QuartzAccelerometer& aclXArg, QuartzAccelerometer& aclYArg, QuartzAccelerometer& aclZArg, GNSS& gnssArg,
             ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg)
        : BlackBox(), rlgPitch(rlgPitchArg), rlgYaw(rlgYawArg), rlgRoll(rlgRollArg),
          aclX(aclXArg), aclY(aclYArg), aclZ(aclZArg), gnss(gnssArg),
          acPower(acPowerArg),
          currentPosition(0.0), alignmentTime(300), isReady(false), currentLatitude(0.0), currentLongitude(0.0), currentCalculatedAltitude(0.0),
          currentBaroAltitude(0.0), currentRadarAltitude(0.0), currentGroundSpeed(0.0), currentTrueAirSpeed(0.0), currentIndicatedAirSpeed(0.0),
          currentCalibratedAirSpeed(0.0), currentAngleOfBank(0.0), currentAngleOfPitch(0.0), currentAngleOfYaw(0.0), currentMagneticHeading(0),
          currentTrueHeading(0), currentSpeedError(100), currentHeadingError(100), systemDamage(0) {}

void BII_9::setCurrentLatitude(const double latitude) {
    currentLatitude = latitude;
}

double BII_9::getCurrentLatitude() const {
    return currentLatitude;
}

void BII_9::setCurrentLongitude(const double longitude) {
    currentLongitude = longitude;
}

double BII_9::getCurrentLongitude() const {
    return currentLongitude;
}

void BII_9::setCurrentHeading(const double heading) {
    currentTrueHeading = heading;
}

double BII_9::getCurrentHeading() const {
    return currentTrueHeading;
}

void BII_9::setCurrentGroundSpeed(const double groundSpeed) {
    currentGroundSpeed = groundSpeed;
}

double BII_9::getCurrentGroundSpeed() const {
    return currentGroundSpeed;
}

void BII_9::setCurrentTAS(const double TAS) {
    currentTrueAirSpeed = TAS;
}

double BII_9::getCurrentTAS() const {
    return currentTrueAirSpeed;
}

float BII_9::getCurrentRadarAltitude() const {
    return currentRadarAltitude;
}

void BII_9::setCurrentRadarAltitude(double altitude) {
    currentRadarAltitude = altitude;
}

float BII_9::getCurrentBaroAltitude() const {
    return currentBaroAltitude;
}

void BII_9::setCurrentBaroAltitude(double altitude) {
    currentBaroAltitude = altitude;
}

int_fast8_t BII_9::getCurrentHeading() const {
    return currentMagneticHeading;
}

// function to update latitude and longitude based on heading and ground speedX
void BII_9::updatePosition() {
    float headingRad = currentTrueHeading * M_PI / 180.0;   //true heading converted to radians
    float deltaLatitude = currentGroundSpeed * refreshInterval.count() / 1000.0 * std::sin(headingRad); // calculate delta lat
    currentLatitude += deltaLatitude;   // update latitude
    float deltaLongitude = currentGroundSpeed * refreshInterval.count() / 1000.0 * std::cos(headingRad); // calculate delta long
    currentLongitude += deltaLongitude; // update longitude
}


// unit tests

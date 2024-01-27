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
    this->currentLatitude = latitude;
}

double BII_9::getCurrentLatitude() const {
    return this->currentLatitude;
}

void BII_9::setCurrentLongitude(const double longitude) {
    this->currentLongitude = longitude;
}

double BII_9::getCurrentLongitude() const {
    return this->currentLongitude;
}

void BII_9::setCurrentTrueHeading(const double heading) {
    this->currentTrueHeading = heading;
}

double BII_9::getCurrentTrueHeading() const {
    return this->currentTrueHeading;
}

void BII_9::setCurrentMagHeading(const double heading) {
    this->currentMagneticHeading = heading;
}

double BII_9::getCurrentMagHeading() const {
    return this->currentMagneticHeading;
}

void BII_9::setCurrentMagneticVariation(const double magVariation) {
    this->currentMagneticVariation = magVariation;
}

double BII_9::getCurrentMagneticVariation() const {
    return this->currentMagneticVariation;
}

void BII_9::setCurrentBaroAltitude(const double altitude) {
    this->currentBaroAltitude = altitude;
}

double BII_9::getCurrentBaroAltitude() const {
    return this->currentBaroAltitude;
}

void BII_9::setCurrentRadarAltitude(const double altitude) {
    this->currentRadarAltitude = altitude;
}

double BII_9::getCurrentRadarAltitude() const {
    return this->currentRadarAltitude;
}

void BII_9::setCurrentAltitudeASL(const double altitude) {
    this->currentAltitudeASL = altitude;
}

double BII_9::getCurrentAltitudeASL() const {
    return this->currentAltitudeASL;
}

void BII_9::setCurrentGroundSpeed(const double speed) {
    this->currentGroundSpeed = speed;
}

double BII_9::getCurrentGroundSpeed() const {
    return this->currentGroundSpeed;
}

void BII_9::setCurrentTAS(const double speed) {
    this->currentTrueAirSpeed = speed;
}

double BII_9::getCurrentTAS() const {
    return this->currentTrueAirSpeed;
}

void BII_9::setCurrentIAS(const double speed) {
    this->currentIndicatedAirSpeed = speed;
}

double BII_9::getCurrentIAS() const {
    return currentIndicatedAirSpeed;
}

void BII_9::setCurrentCAS(const double speed) {
    this->currentCalibratedAirSpeed = speed;
}

double BII_9::getCurrentCAS() const {
    return this->currentCalibratedAirSpeed;
}

void BII_9::setCurrentAngleOfBank(const double angle) {
    this->currentAngleOfBank = angle;
}

double BII_9::getCurrentAngleOfBank() const {
    return currentAngleOfBank;
}

void BII_9::setCurrentAngleOfPitch(const double angle) {
    this->currentAngleOfPitch = angle;
}

double BII_9::getCurrentAngleOfPitch() const {
    return this->currentAngleOfPitch;
}

void BII_9::updatePosition() {
    double deltaX = aclX.getAcceleration(); // forward/backward acceleration
    double deltaY = aclY.getAcceleration(); // left/right acceleration
    double deltaZ = aclZ.getAcceleration(); // up/down acceleration
    double deltaPitch = rlgPitch.updateOrientation(currentAngleOfPitch, dt);
    double deltaHeading = rlgYaw.updateOrientation(currentTrueHeading, dt);
    double deltaRoll = rlgRoll.updateOrientation(currentAngleOfBank, dt);

}





// unit tests

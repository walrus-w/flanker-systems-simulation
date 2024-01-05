//
// Created by Charles (Walrus) on 03/01/2024.
//

/*
 * Derived class from BlackBox
 * Initial implementation will be a simple representation of a modern INS, using three RLGs and three accelerometers.
 */

#ifndef SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX
#define SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <cmath>
#include "ring-laser-gyroscope.hxx"
#include "../black-box.hxx"
#include "accelerometer.hxx"
#include "ac-power-supply.hxx"
#include "dc-power-supply.hxx"
#include "gnss.hxx"

class InertialNavigationSystem:BlackBox {
public:

    InertialNavigationSystem(RingLaserGyroscope& pitch, RingLaserGyroscope& yaw, RingLaserGyroscope& roll);
    std::chrono::seconds getAlignmentTime();
    float getCurrentLat();
    float getCurrentLong();
    float getCurrentSpeed();
    float getCurrentRadarAltitude();
    float getCurrentBaroAltitude();
    int_fast8_t getCurrentHeading();

private:

    // data members
    //gyros measuring changes in pitch, roll and yaw axes
    RingLaserGyroscope& rlgPitch;
    RingLaserGyroscope& rlgYaw;
    RingLaserGyroscope& rlgRoll;
    // accelerometers measuring changes in velocity in x, y and z planes
    // i.e. forward-aft, side-to-side and up-down respectively
    Accelerometer aclX;
    Accelerometer aclY;
    Accelerometer aclZ;
    //
    GNSS gnss;
    // Su-27SK and MiG-29G documentation both show use of an AC and DC power supply for INS
    // Which is used to power what elements is still unclear; searching for format to translate Su-27SK tech manual
    ACPowerSupply acPower;
    DCPowerSupply dcPower;

    float currentPosition;          // current position is stored in decimal degrees
    std::chrono::seconds alignmentTime;          // time remaining for full alignment of INS
    bool isReady;                   // true if INS is aligned sufficiently for flight and weapons employment
    float currentLatitude;          // current lat/long of system. Initialised to nullptr
    float currentLongitude;
    float currentBaroAltitude;
    float currentRadarAltitude;
    float currentGroundSpeed;
    float currentTrueAirSpeed;
    float currentIndicatedAirSpeed;
    float currentCalibratedAirSpeed;
    float currentAngleOfBank;
    float currentAngleOfPitch;
    float currentAngleOfYaw;
    uint_least8_t currentMagneticHeading;
    uint_least8_t currentTrueHeading;
    uint_least8_t currentSpeedError;    // initialised to an arbitrary maximum value, which decreases as alignment time increases
    uint_least8_t currentHeadingError;  // error will gradually accumulate post-alignment, unless corrected via satellite or (potentially) radar fix
    uint_least8_t systemDamage;         // placeholder, not presently used
    std::chrono::milliseconds refreshInterval = 10  // interval between system state updates; 100 times per second

    // function members
    // helper functions
    void setLatitude(float lat);
    void setLongitude(float longitude);
    void setHeading(int_least8_t heading);
    void setAltitude(float altitude);
    float getCurrentLat();
    void setLatitude(float lat);
    float getCurrentLong();
    void setLongitude(float longitude);
    float getCurrentSpeed();
    void setCurrentSpeed(float speed);
    float getCurrentRadarAltitude();
    void setCurrentRadarAltitude(float altitude);
    float getCurrentBaroAltitude();
    void setCurrentBaroAltitude(float altitude);
    int_fast8_t getCurrentHeading();
    void setCurrentHeading(int_least8_t heading);

    void updatePosition(); // function to update latitude and longitude based on heading and ground speed

};

InertialNavigationSystem::InertialNavigationSystem(RingLaserGyroscope& pitch, RingLaserGyroscope& yaw, RingLaserGyroscope& roll)
        : rlgPitch(pitch), rlgYaw(yaw), rlgRoll(roll),
          aclX(), aclY(), aclZ(),
          gnss(),
          acPower(115, 400), dcPower(27),   //400Hz figure from Anuva Technologies Su-30MKI cockpit displays
          currentPosition(0.0),
          alignmentTime(300),
          isReady(false),
          currentLatitude(0.0),
          currentLongitude(0.0),
          currentBaroAltitude(0.0),
          currentRadarAltitude(0.0),
          currentGroundSpeed(0.0),
          currentTrueAirSpeed(0.0),
          currentIndicatedAirSpeed(0.0),
          currentCalibratedAirSpeed(0.0),
          currentAngleOfBank(0.0),
          currentAngleOfPitch(0.0),
          currentAngleOfYaw(0.0),
          currentMagneticHeading(0),
          currentTrueHeading(0),
          currentSpeedError(100),
          currentHeadingError(100),
          systemDamage(0);

float InertialNavigationSystem::getCurrentLat() {
    return currentLatitude;
}

void InertialNavigationSystem::setLatitude(float lat) {
    currentLatitude = lat;
}

float InertialNavigationSystem::getCurrentLong() {
    return currentLongitude;
}

void InertialNavigationSystem::setLongitude(float longitude) {
    currentLongitude = longitude;
}

float InertialNavigationSystem::getCurrentGroundSpeed() {
    return currentGroundSpeed;
}

void InertialNavigationSystem::setCurrentGroundSpeed(float speed) {
    currentGroundSpeed = speed;
}

float InertialNavigationSystem::getCurrentRadarAltitude() {
    return currentRadarAltitude;
}

void InertialNavigationSystem::setCurrentRadarAltitude(float altitude) {
    currentRadarAltitude = altitude;
}

float InertialNavigationSystem::getCurrentBaroAltitude() {
    return currentBaroAltitude;
}

void InertialNavigationSystem::setCurrentBaroAltitude(float altitude) {
    currentBaroAltitude = altitude;
}

int_fast8_t InertialNavigationSystem::getCurrentHeading() {
    return currentMagneticHeading;
}

void InertialNavigationSystem::setCurrentHeading(int_least8_t heading) {
    currentMagneticHeading = heading;
}

// function to update latitude and longitude based on heading and ground speed
void InertialNavigationSystem::updatePosition() {
    float headingRad = currentTrueHeading * M_PI / 180.0;   //true heading converted to radians
    float deltaLatitude = currentGroundSpeed * refreshInterval.count() / 1000.0 * std::sin(headingRad); // calculate delta lat
    currentLatitude += deltaLatitude;   // update latitude
    float deltaLongitude = currentGroundSpeed * refreshInterval.count() / 1000.0 * std::cos(headingRad); // calculate delta long
    currentLongitude += deltaLongitude; // update longitude
}





#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

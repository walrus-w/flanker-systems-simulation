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
    float getCurrentAltitude();
    float getCurrentHeading();

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
    GNSS;
    // Su-27SK and MiG-29G documentation both show use of an AC and DC power supply for INS
    // Which is used to power what elements is still unclear
    ACPowerSupply acPower;
    DCPowerSupply dcPower;

    float currentPosition;          // current position is stored in decimal degrees
    std::chrono::seconds alignmentTime;          // time remaining for full alignment of INS
    bool isReady;                   // true if INS is aligned sufficiently for flight and weapons employment
    float currentLatitude;          // current lat/long of system. Initialised to nullptr
    float currentLongitude;
    float currentAltitude;
    float currentGroundSpeed;
    float currentTrueAirSpeed;
    float currentIndicatedAirSpeed;
    float currentCalibratedAirSpeed;
    uint_least8_t currentHeading;
    uint_least8_t currentSpeedError;    // initialised to an arbitrary maximum value, which decreases as alignment time increases
    uint_least8_t currentHeadingError;  // error will gradually accumulate post-alignment, unless corrected via satellite or (potentially) radar fix
    uint_least8_t systemDamage;         // placeholder, not presently used

    // function members
    void setLatitude(float lat);        // functions used to initialise INS position and heading
    void setLongitude(float longitude);
    void setHeading(int_least8_t heading);
    void setAltitude(float altitude);

};

InertialNavigationSystem::InertialNavigationSystem(RingLaserGyroscope& pitch, RingLaserGyroscope& yaw, RingLaserGyroscope& roll)
        : rlgPitch(pitch), rlgYaw(yaw), rlgRoll(roll),
          aclX(), aclY(), aclZ(),
          acPower(120), dcPower(29),
          currentPosition(0.0),
          alignmentTime(300),
          isReady(false),
          currentLatitude(0.0),
          currentLongitude(0.0),
          currentAltitude(0.0),
          currentGroundSpeed(0.0),
          currentTrueAirSpeed(0.0),
          currentIndicatedAirSpeed(0.0),
          currentCalibratedAirSpeed(0.0),
          currentHeading(0),
          currentSpeedError(100),
          currentHeadingError(100),
          systemDamage(0)

void InertialNavigationSystem::setLatitude(float lat) {
    currentLatitude = lat;
}

void InertialNavigationSystem::setLongitude(float longitude) {
    currentLongitude = longitude;
}

void InertialNavigationSystem::setHeading(uint_least8_t heading) {
    currentHeading = heading;
}

void InertialNavigationSystem::setAltitude(float altitude) {
    currentAltitude = altitude;
}




#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

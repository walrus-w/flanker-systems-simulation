//
// Created by ctobi on 3/01/2024.
//

/*
 * Derived from BlackBox
 * Initial implementation will be a simple representation of a modern INS, using RLGs.
 */

#ifndef SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX
#define SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

#include <cstddef>
#include <cstdint>
#include <chrono>
#include "ring-laser-gyroscope.hxx"
#include "../black-box.hxx"
#include "accelerometer.hxx"

class InertialNavigationSystem:BlackBox {
public:

    InertialNavigationSystem();
    seconds getAlignmentTime();
    float getCurrentLat();
    float getCurrentLong();
    float getCurrentSpeed();
    float getCurrentAltitude();
    float getCurrentHeading();

private:

    // data members
    RingLaserGyroscope rlgPitch;    //gyros measuring changes in pitch, roll and yaw axes
    RingLaserGyroscope rlgYaw;
    RingLaserGyroscope rlgRoll;
    Accelerometer aclX;             // accelerometers measuring changes in velocity in x, y and z planes
    Accelerometer aclY;             // i.e. forward-aft, side-to-side and up-down respectively
    Accelerometer aclZ;

    float currentPosition;          // current position is stored in decimal degrees
    seconds alignmentTime;          // time remaining for full alignment of INS
    bool isReady;                   // true if INS is aligned sufficiently for flight and weapons employment
    float currentLatitude;          // current lat/long of system. Initialised to nullptr
    float currentLongitude;
    float currentAltitude;
    float currentSpeed;
    int_least8_t currentHeading;
    uint_least8_t currentSpeedError;    // initialised to an arbitrary maximum value, which decreases as alignment time increases
    uint_least8_t currentHeadingError;  // error will gradually accumulate post-alignment, unless corrected via satellite or (potentially) radar fix

    // function members
    void setLatitude(float lat);        // functions used to initialise INS position and heading
    void setLongitude(float longitude);
    void setHeading(int_least8_t);
    void setAltitude(float altitude)

};

InertialNavigationSystem::InertialNavigationSystem() {
    rlgPitch = new RingLaserGyroscope();
    rlgYaw = new RingLaserGyroscope();
    rlgRoll = new RingLaserGyroscope();
    aclX = new Accelerometer();
    aclY = new Accelerometer();
    aclZ = new Accelerometer();
    satelliteAntenna = true;
    alignmentTime = 300;
    currentSpeedError = 100;
    currentHeadingError = 100;
    currentLatitude = nullptr;
    currentLongitude = nullptr;
    currentAltitude = nullptr;
    currentSpeed = nullptr;
}

void InertialNavigationSystem::setLatitude(float lat) {
    currentLatitude = lat;
}

void InertialNavigationSystem::setLongitude(float longitude) {
    currentLongitude = longitude;
}




#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

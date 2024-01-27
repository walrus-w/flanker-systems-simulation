//
// Created by Charles (Walrus) on 03/01/2024.
//

/*
 * Derived class from BlackBox
 * Initial implementation will be a simple representation of a modern INS, using three RLGs and three accelerometers.
 * NavigationComputer class interfaces between INS and cockpit displays
 * Pitot and static pressure measurements are passed through NavigationComputer for cockpit displays and instruments
 * This is a simplified version which does not use a Kalman filter algorithm to
 */

#ifndef SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX
#define SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <numbers>
#include <cmath>
#include "ringLaserGyroscopeV2.hpp"
#include "blackBox.hpp"
#include "quartzAccelerometer.hpp"
#include "acPowerSupply.hpp"
#include "dcPowerSupply.hpp"
#include "gnss.hpp"

class BII_9: BlackBox {
public:

    // cold and dark cockpit without stored position constructor
    // consider parameterizing all members in single constructor and initialise variables using cockpitInit function
    BII_9(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
          RingLaserGyroscope& rlgRollArg, QuartzAccelerometer& aclXArg, QuartzAccelerometer& aclYArg,
          QuartzAccelerometer& aclZArg, GNSS& gnssArg, ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg);

    // cold and dark cockpit with stored position constructor
    BII_9(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
          RingLaserGyroscope& rlgRollArg, QuartzAccelerometer& aclXArg, QuartzAccelerometer& aclYArg,
          QuartzAccelerometer& aclZArg, GNSS& gnssArg, ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg,
          bool ready = true);

    // hot ground start constructor
    BII_9(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
          RingLaserGyroscope& rlgRollArg, QuartzAccelerometer& aclXArg, QuartzAccelerometer& aclYArg,
          QuartzAccelerometer& aclZArg, GNSS& gnssArg, ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg,
          double latArg, double longArg, double currentCalculatedAltitude);

private:

    // data members
    //gyros measuring changes in pitch, roll and yaw axes
    RingLaserGyroscope& rlgPitch;
    RingLaserGyroscope& rlgYaw;
    RingLaserGyroscope& rlgRoll;
    // accelerometers measuring changes in velocity in x, y and z planes
    // i.e. forward-aft, side-to-side and up-down respectively
    QuartzAccelerometer& aclX;
    QuartzAccelerometer& aclY;
    QuartzAccelerometer& aclZ;
    //
    GNSS& gnss;
    // Su-27SK and MiG-29G documentation both show use of an AC and DC power supply for INS
    // OEM sales material seems to indicate the modern system runs only from the AC system
    ACPowerSupply& acPower; //400Hz figure from Anuva Technologies Su-30MKI cockpit displays

    // positional and orientation variables
    double currentLatitude;          // current lat/long of system. Initialised to nullptr
    double currentLongitude;
    double currentMagneticHeading;
    double currentTrueHeading;
    double currentBaroAltitude;
    double currentRadarAltitude;
    double currentAltitudeASL;
    double currentGroundSpeed;
    double currentTrueAirSpeed;
    double currentIndicatedAirSpeed;
    double currentCalibratedAirSpeed;
    double currentAngleOfBank;
    double currentAngleOfPitch;
    double currentAngleOfYaw;
    double currentSpeedError;    // initialised to an arbitrary maximum value, which decreases as alignment time increases
    double currentHeadingError;  // error will gradually accumulate post-alignment, unless corrected via satellite or (potentially) radar fix

   // physical system variables
    std::chrono::seconds alignmentTime;          // time remaining for full alignment of INS
    bool isReady;                   // true if INS is aligned sufficiently for flight and weapons employment
    double systemDamage;         // placeholder, not presently used

    // interval between system state updates; 100 times per second
    const std::chrono::milliseconds refreshInterval = std::chrono::milliseconds(20);
    const int dt = refreshInterval.count() / 20.0; // Time step in 20 ms intervals

    // physical constants
    // mean Earth radius in meters
    const double Re = 6371000;

    // function members
    // system initialisation functions
    void setCurrentLatitude(double latitude);
    [[nodiscard]] double getCurrentLatitude() const;
    void setCurrentLongitude(double longitude);
    [[nodiscard]] double getCurrentLongitude() const;
    void setCurrentHeading(double heading);
    [[nodiscard]] double getCurrentHeading() const;
    void setCurrentGroundSpeed(double groundSpeed);
    [[nodiscard]] double getCurrentGroundSpeed() const;
    void setCurrentTAS(double tas);
    [[nodiscard]] double getCurrentTAS() const;
    void setAltitude(double altitude);
    void setCurrentGroundSpeed(double speed);
    void setCurrentRadarAltitude(double altitude);
    void setCurrentBaroAltitude(double altitude);

};



#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

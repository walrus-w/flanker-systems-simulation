//
// Created by Charles (Walrus) on 03/01/2024.
//

/*
 * Derived class from BlackBox
 * Initial implementation will be a simple representation of a modern INS, using three RLGs and three accelerometers.
 * NavigationComputer class interfaces between INS and cockpit displays
 * Pitot and static pressure measurements are passed through NavigationComputer for cockpit displays and instruments
 */

#ifndef SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX
#define SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

#define _USE_MATH_DEFINES

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <numbers>
#include <cmath>
#include "ringLaserGyroscopeV2.hpp"
#include "blackBox.hpp"
#include "accelerometer.hpp"
#include "acPowerSupply.hpp"
#include "dcPowerSupply.hpp"
#include "gnss.hpp"
#include <Eigen/Dense> // Include Eigen library for linear algebra operations

class InertialNavigationSystem:BlackBox {
public:

    // cold and dark cockpit without stored position constructor
    // consider parameterizing all members in single constructor and initialise variables using cockpitInit function
    InertialNavigationSystem(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
                             RingLaserGyroscope& rlgRollArg, Accelerometer& aclXArg, Accelerometer& aclYArg,
                             Accelerometer& aclZArg, GNSS& gnssArg, ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg);

    // cold and dark cockpit with stored position constructor
    InertialNavigationSystem(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
                             RingLaserGyroscope& rlgRollArg, Accelerometer& aclXArg, Accelerometer& aclYArg,
                             Accelerometer& aclZArg, GNSS& gnssArg, ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg,
                             bool ready = true);

    // hot ground start constructor
    InertialNavigationSystem(RingLaserGyroscope& rlgPitchArg, RingLaserGyroscope& rlgYawArg,
                             RingLaserGyroscope& rlgRollArg, Accelerometer& aclXArg, Accelerometer& aclYArg,
                             Accelerometer& aclZArg, GNSS& gnssArg, ACPowerSupply& acPowerArg, DCPowerSupply& dcPowerArg,
                             float latArg, float longArg, float currentCalculatedAltitude);

    std::chrono::seconds getAlignmentTime();
    float getCurrentLat();
    float getCurrentLong();
    float getCurrentGroundSpeed();
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
    Accelerometer& aclX;
    Accelerometer& aclY;
    Accelerometer& aclZ;
    //
    GNSS& gnss;
    // Su-27SK and MiG-29G documentation both show use of an AC and DC power supply for INS
    // Which is used to power what elements is still unclear; searching for format to translate Su-27SK tech manual
    ACPowerSupply& acPower; //400Hz figure from Anuva Technologies Su-30MKI cockpit displays
    DCPowerSupply& dcPower;

    // positional and orientation variables
    float currentPosition;          // current position is stored in decimal degrees
    float currentLatitude;          // current lat/long of system. Initialised to nullptr
    float currentLongitude;
    float currentCalculatedAltitude;
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

    // Kalman Filter variables
    Eigen::MatrixXd state;    // State vector [latitude, longitude, ground speed, heading]
    Eigen::MatrixXd covariance; // Covariance matrix

    // Kalman filter function parameters
    Eigen::MatrixXd A;   // state transition matrix
    Eigen::MatrixXd H;   // measurement matrix
    Eigen::MatrixXd Q;   // process noise covariance
    Eigen::MatrixXd R;   // measurement noise covariance

    // physical system variables
    std::chrono::seconds alignmentTime;          // time remaining for full alignment of INS
    bool isReady;                   // true if INS is aligned sufficiently for flight and weapons employment
    uint_least8_t systemDamage;         // placeholder, not presently used

    // interface variables
    const std::chrono::milliseconds refreshInterval = std::chrono::milliseconds(10);;  // interval between system state updates; 100 times per second

    // function members
    // helper functions
    void setLatitude(float lat);
    void setLongitude(float longitude);
    void setHeading(int_least8_t heading);
    void setAltitude(float altitude);
    void setCurrentGroundSpeed(float speed);
    void setCurrentRadarAltitude(float altitude);
    void setCurrentBaroAltitude(float altitude);
    void setCurrentHeading(int_least8_t heading);

    // system initialisation functions
    // parameters are placeholders pending API integration



    // Kalman filter implementation functions
    const float dt = refreshInterval.count() / 20.0; // Time step in 20 ms intervals
    void initializeNoiseCovarianceMatrices(); // Process noise parameters (standard deviations)
    void initializeKalmanFilter();   // initialises Kalman filter variables to simulate very low uncertainty in system
    void updateWithGNSSMeasurements(Eigen::VectorXd& z);
    void estimateStateChange();
    void estimateVelocityChange(float accelInX, float accelInY, float accelInZ);  // estimate velocity change using Kalman filter function
    // estimateVelocityChange helper functions
    void predictStep();
    void updateStateWithAccelMeasurement(float accelInX, float accelInY, float accelInZ);
    void updateStep();
    void updateCurrentVariables();
    void updateStateWithGyroMeasurement(float_t gyroPitchRate, float_t gyroYawRate, float_t gyroRollRate);
    void estimatePositionChange();  // estimate position change using velocity change a Kalman filter
    void updatePosition(); // function to update latitude and longitude based on heading and ground speed

};



#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

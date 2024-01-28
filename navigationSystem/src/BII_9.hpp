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
#include <Eigen/Dense> // Include Eigen library for linear algebra operations

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

    std::chrono::seconds getAlignmentTime();
    [[nodiscard]] double getCurrentLat() const;
    [[nodiscard]] double getCurrentLong() const;
    [[nodiscard]] double getCurrentGroundSpeed() const;
    [[nodiscard]] double getCurrentRadarAltitude() const;
    [[nodiscard]] double getCurrentBaroAltitude() const;
    [[nodiscard]] double getCurrentHeading() const;

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
    // Which is used to power what elements is still unclear; searching for format to translate Su-27SK tech manual
    ACPowerSupply& acPower; //400Hz figure from Anuva Technologies Su-30MKI cockpit displays
    DCPowerSupply& dcPower;

    // positional and orientation variables
    double currentPosition;          // current position is stored in decimal degrees
    double currentLatitude;          // current lat/long of system. Initialised to nullptr
    double currentLongitude;
    double currentCalculatedAltitude;
    double currentBaroAltitude;
    double currentRadarAltitude;
    double currentGroundSpeed;
    double currentTrueAirSpeed;
    double currentIndicatedAirSpeed;
    double currentCalibratedAirSpeed;
    double currentAngleOfBank;
    double currentAngleOfPitch;
    double currentAngleOfYaw;
    double currentMagneticHeading;
    double currentTrueHeading;
    double currentSpeedError;    // initialised to an arbitrary maximum value, which decreases as alignment time increases
    double currentHeadingError;  // error will gradually accumulate post-alignment, unless corrected via satellite or (potentially) radar fix

    // Kalman Filter variables
    Eigen::MatrixXd state;    // State vector [latitude, longitude, ground speedX, heading]
    Eigen::MatrixXd covariance; // Covariance matrix

    // Kalman filter function parameters
    Eigen::MatrixXd A;   // state transition matrix
    Eigen::MatrixXd H;   // measurement matrix
    Eigen::MatrixXd Q;   // process noise covariance
    Eigen::MatrixXd R;   // measurement noise covariance

    // physical system variables
    std::chrono::seconds alignmentTime;          // time remaining for full alignment of INS
    bool isReady;                   // true if INS is aligned sufficiently for flight and weapons employment
    double systemDamage;         // placeholder, not presently used

    // interface variables
    const std::chrono::milliseconds refreshInterval = std::chrono::milliseconds(20);;  // interval between system state updates; 100 times per second

    // physical constants
    // mean Earth radius in meters
    const double Re = 6371000;


    // function members
    // helper functions
    void setLatitude(double lat);
    void setLongitude(double longitude);
    void setHeading(double heading);
    void setAltitude(double altitude);
    void setCurrentGroundSpeed(double speed);
    void setCurrentRadarAltitude(double altitude);
    void setCurrentBaroAltitude(double altitude);
    void setCurrentHeading(double heading);

    // system initialisation functions
    // parameters are placeholders pending API integration



    // Kalman filter implementation functions
    const long dt = refreshInterval.count() / 20.0; // Time step in 20 ms intervals
    void initializeNoiseCovarianceMatrices(); // Process noise parameters (standard deviations)
    void initializeKalmanFilter();   // initialises Kalman filter variables to simulate very low uncertainty in system
    Eigen::MatrixXd initializeMatrix(bool initializeToZero, int rows, int cols, float scaleFactor = 1);
    void updateWithGNSSMeasurements(Eigen::VectorXd& z);
    void estimateStateChange();
    void estimateVelocityChange(double accelInX, double accelInY, double accelInZ);  // estimate velocity change using Kalman filter function
    // estimateVelocityChange helper functions
    void predictStep();
    void updateStateWithAccelMeasurement(double accelInX, double accelInY, double accelInZ);
    void updateStep();
    void updateCurrentVariables();
    void updateStateWithGyroMeasurement(double gyroPitchRate, double gyroYawRate, double gyroRollRate);
    void estimatePositionChange();  // estimate position change using velocity change a Kalman filter
    void updatePosition(); // function to update latitude and longitude based on heading and ground speedX

};



#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

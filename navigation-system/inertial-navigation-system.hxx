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
#include "black-box.hxx"
#include "accelerometer.hxx"
#include "ac-power-supply.hxx"
#include "dc-power-supply.hxx"
#include "gnss.hxx"
#include <Eigen/Dense>  // Include Eigen library for linear algebra operations

class InertialNavigationSystem:BlackBox {
public:

    InertialNavigationSystem(RingLaserGyroscope& pitch, RingLaserGyroscope& yaw, RingLaserGyroscope& roll);
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
    Accelerometer aclX;
    Accelerometer aclY;
    Accelerometer aclZ;
    //
    GNSS gnss;
    // Su-27SK and MiG-29G documentation both show use of an AC and DC power supply for INS
    // Which is used to power what elements is still unclear; searching for format to translate Su-27SK tech manual
    ACPowerSupply acPower;
    DCPowerSupply dcPower;

    // positional and orientation variables
    float currentPosition;          // current position is stored in decimal degrees
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

    // Kalman Filter variables
    void initializeKalmanFilter();   // initialises Kalman filter variables to simulate very low uncertainty in system
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

    // navigation functions
    void updatePosition(); // function to update latitude and longitude based on heading and ground speed
    void estimateVelocityChange(float accelInX, float accelInY, float accelInZ);  // estimate velocity change using Kalman filter function
    void estimatePositionChange();  // estimate position change using velocity change a

};

// constructor for cold and dark aircraft
InertialNavigationSystem::InertialNavigationSystem(RingLaserGyroscope& pitch, RingLaserGyroscope& yaw, RingLaserGyroscope& roll)
    : rlgPitch(pitch), rlgYaw(yaw), rlgRoll(roll),
            aclX(), aclY(), aclZ(),
            gnss(),
            acPower(115, 400), dcPower(27),   //400Hz figure from Anuva Technologies Su-30MKI cockpit displays
            currentPosition(0.0), alignmentTime(300), isReady(false), currentLatitude(0.0), currentLongitude(0.0), currentBaroAltitude(0.0),
            currentRadarAltitude(0.0), currentGroundSpeed(0.0), currentTrueAirSpeed(0.0), currentIndicatedAirSpeed(0.0), currentCalibratedAirSpeed(0.0),
            currentAngleOfBank(0.0), currentAngleOfPitch(0.0), currentAngleOfYaw(0.0), currentMagneticHeading(0), currentTrueHeading(0),
            currentSpeedError(100), currentHeadingError(100), systemDamage(0) {}

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

void InertialNavigationSystem::initializeKalmanFilter() {
    // initialize Kalman filter matrices and parameters
    state = Eigen::MatrixXd::Zero(5, 1); // initialize state vector to zeros
    covariance = Eigen::MatrixXd::Identity(5, 5); // initialize covariance matrix to identity

    // Set Kalman filter parameters
    A = Eigen::MatrixXd::Identity(5, 5); // state transition matrix
    H = Eigen::MatrixXd::Identity(5, 5); // measurement matrix

    // set very low process noise covariance to simulate high accuracy in system dynamics
    Q = Eigen::MatrixXd::Identity(5, 5) * 1e-6;

    // set very low measurement noise covariance to simulate high accuracy in sensor measurements
    R = Eigen::MatrixXd::Identity(5, 5) * 1e-6;
}

void InertialNavigationSystem::estimateVelocityChange(float accelInX, float accelInY, float accelInZ) {
    // Kalman filter prediction step
    state = A * state;             // predict state
    covariance = A * covariance * A.transpose() + Q;  // predict covariance

    // Update state based on accelerometer measurements
    float dt = refreshInterval.count() / 1000.0;  // time step in seconds
    state(0, 0) += state(2, 0) * dt;  // update latitude based on current ground speed
    state(1, 0) += state(3, 0) * dt;  // update longitude based on current ground speed

    // Kalman filter update step
    Eigen::MatrixXd measurement(6, 1);  // measurement vector [latitude, longitude, ground speed, accelInX, accelInY, accelInZ]
    measurement << currentLatitude, currentLongitude, currentGroundSpeed, accelInX, accelInY, accelInZ;
    Eigen::MatrixXd innovation = measurement - H * state;  // Innovation
    Eigen::MatrixXd innovationCovariance = H * covariance * H.transpose() + R;  // Innovation covariance
    Eigen::MatrixXd kalmanGain = covariance * H.transpose() * innovationCovariance.inverse();  // Kalman gain

    // Update state and covariance
    state = state + kalmanGain * innovation;
    covariance = (Eigen::MatrixXd::Identity(6, 6) - kalmanGain * H) * covariance;

    // Update current latitude, longitude, and ground speed with Kalman filter estimates
    currentLatitude = state(0, 0);
    currentLongitude = state(1, 0);
    currentGroundSpeed = state(2, 0);

    // Update heading using gyroscope measurements (pitch, yaw, roll)
    state(4, 0) += gyroscopePitch.getAngularRate() * dt;  // Update pitch
    state(5, 0) += gyroscopeYaw.getAngularRate() * dt;    // Update yaw
    state(6, 0) += gyroscopeRoll.getAngularRate() * dt;   // Update roll

}

void InertialNavigationSystem::estimatePositionChange() {
    // Use the velocity change function to update the position
    estimateVelocityChange(aclX.getAcceleration(), aclY.getAcceleration(), aclZ.getAcceleration());

    // Kalman filter prediction step for position only
    state.block(0, 0, 2, 1) = A.block(0, 0, 2, 2) * state.block(0, 0, 2, 1);  // Predict latitude and longitude
    covariance.block(0, 0, 2, 2) = A.block(0, 0, 2, 2) * covariance.block(0, 0, 2, 2) * A.block(0, 0, 2, 2).transpose() + Q.block(0, 0, 2, 2);  // Predict covariance

    // Update current latitude and longitude based on ground speed
    float dt = refreshInterval.count() / 1000.0;  // Time step in seconds
    state(0, 0) += state(2, 0) * dt;  // Update latitude based on current ground speed
    state(1, 0) += state(3, 0) * dt;  // Update longitude based on current ground speed

    // Note: This is a simple models and variables will likely need adjustment on the basis of testing
}

#endif //SU_30_EFM_V2_7_3B_INERTIAL_NAVIGATION_SYSTEM_HXX

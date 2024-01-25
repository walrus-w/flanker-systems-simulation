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
          acPower(acPowerArg), dcPower(dcPowerArg),
          currentPosition(0.0), alignmentTime(300), isReady(false), currentLatitude(0.0), currentLongitude(0.0), currentCalculatedAltitude(0.0),
          currentBaroAltitude(0.0), currentRadarAltitude(0.0), currentGroundSpeed(0.0), currentTrueAirSpeed(0.0), currentIndicatedAirSpeed(0.0),
          currentCalibratedAirSpeed(0.0), currentAngleOfBank(0.0), currentAngleOfPitch(0.0), currentAngleOfYaw(0.0), currentMagneticHeading(0),
          currentTrueHeading(0), currentSpeedError(100), currentHeadingError(100), systemDamage(0) {}


double BII_9::getCurrentLat() const {
    return currentLatitude;
}

void BII_9::setLatitude(double lat) {
    currentLatitude = lat;
}

double BII_9::getCurrentLong() const {
    return currentLongitude;
}

void BII_9::setLongitude(double longitude) {
    currentLongitude = longitude;
}

double BII_9::getCurrentGroundSpeed() const {
    return currentGroundSpeed;
}

void BII_9::setCurrentGroundSpeed(double speed) {
    currentGroundSpeed = speed;
}

double BII_9::getCurrentRadarAltitude() const {
    return currentRadarAltitude;
}

void BII_9::setCurrentRadarAltitude(double altitude) {
    currentRadarAltitude = altitude;
}

double BII_9::getCurrentBaroAltitude() const {
    return currentBaroAltitude;
}

void BII_9::setCurrentBaroAltitude(double altitude) {
    currentBaroAltitude = altitude;
}

double BII_9::getCurrentHeading() const {
    return currentMagneticHeading;
}

void BII_9::setCurrentHeading(double heading) {
    currentMagneticHeading = heading;
}

// function to update latitude and longitude based on heading and ground speedX
void BII_9::updatePosition() {
    double headingRad = currentTrueHeading * M_PI / 180.0;   //true heading converted to radians
    double deltaLatitude = currentGroundSpeed * refreshInterval.count() / 1000.0 * std::sin(headingRad); // calculate delta lat
    currentLatitude += deltaLatitude;   // update latitude
    double deltaLongitude = currentGroundSpeed * refreshInterval.count() / 1000.0 * std::cos(headingRad); // calculate delta long
    currentLongitude += deltaLongitude; // update longitude
}

// initializes a MatrixXd object with either 0 or identity matrix
// depending on the 'initializeToZero' flag and optionally scales it by scaleFactor
Eigen::MatrixXd BII_9::initializeMatrix(bool initializeToZero, int rows, int cols, float scaleFactor) {
    if (initializeToZero)
        return Eigen::MatrixXd::Zero(rows, cols);
    else
        return Eigen::MatrixXd::Identity(rows, cols) * scaleFactor;
}

void BII_9::initializeNoiseCovarianceMatrices() {
    // Process noise parameters (standard deviations)
    float pos_stddev_km_hr = 1.85;   //1.85 km/hr for position
    float speed_stddev_m_s = 1.0;    //1 m/s for ground speedX
    float heading_stddev_deg = 0.05; //0.05 deg for heading
    float roll_stddev_deg = 0.05;    //0.05 deg for roll
    float pitch_stddev_deg = 0.05;   //0.05 deg for pitch

    // Typically for process noise covariance Q you would transform stddev to variance by squaring it
    // Here it is assumed stddev^2 is applied directly to give a good weight to each parameter
    Q.block(0, 0, 3, 3) = pos_stddev_km_hr * pos_stddev_km_hr * Eigen::MatrixXd::Identity(3, 3); //applying to position
    Q.block(3, 3, 2, 2) = speed_stddev_m_s * speed_stddev_m_s * Eigen::MatrixXd::Identity(2, 2); //applying to velocity
    Q.block(5, 5, 1, 1) = heading_stddev_deg * heading_stddev_deg * Eigen::MatrixXd::Identity(1, 1); //applying to heading
    Q.block(6, 6, 2, 2) = roll_stddev_deg * roll_stddev_deg * Eigen::MatrixXd::Identity(2, 2); //applying to roll & pitch

    // Measurement noise parameters (assumed, could vary depending on actual system)
    float measure_noise_pos = 1.0; // Position measurement noise
    float measure_noise_vel = 0.1; // Velocity measurement noise
    float measure_noise_ang = 0.01; // Angle measurement noise

    // Typically for measurement noise covariance R you would transform stddev to variance by squaring it
    // Here it is assumed stddev^2 is applied directly to give a good weight to each parameter
    R.block(0, 0, 3, 3) = measure_noise_pos * measure_noise_pos * Eigen::MatrixXd::Identity(3, 3); //applying to position
    R.block(3, 3, 2, 2) = measure_noise_vel * measure_noise_vel * Eigen::MatrixXd::Identity(2, 2); //applying to velocity
    R.block(5, 5, 2, 2) = measure_noise_ang * measure_noise_ang * Eigen::MatrixXd::Identity(2, 2); //applying to angles
}

// initialises a 9 x 9 Kalman filter matrix;
// OEM documentation describes using a 32nd order filter so clearly there are a large number of input channels of which information is not available
// current implementation uses six elements of the vector, but the implementation supports extension to include additional arguments
void BII_9::initializeKalmanFilter() {
    // initialize state vector to zeros
    state = Eigen::MatrixXd::Zero(9, 1);

    // Initialize covariance matrix with high initial uncertainty (identity matrix)
    covariance = Eigen::MatrixXd::Identity(9, 9);

    // Initialize transition and measurement matrices
    A = Eigen::MatrixXd::Identity(9, 9);
    H = Eigen::MatrixXd::Identity(9, 9);

    // Initialize process noise covariance Q and measurement noise covariance R
    initializeNoiseCovarianceMatrices();
}

void BII_9::updateWithGNSSMeasurements(Eigen::VectorXd& z) {
    // Calculate the innovation vector (difference between observed and predicted state)
    Eigen::VectorXd y = z - H * state;

    // Calculate the innovation covariance matrix
    Eigen::MatrixXd S = H * covariance * H.transpose() + R;

    // Calculate the optimal Kalman gain
    Eigen::MatrixXd K = covariance * H.transpose() * S.inverse();

    // Update the state vector estimate
    state += K * y;

    // Update the covariance matrix estimate
    int size = covariance.rows();  // Getting size of covariance matrix (assuming it's square)
    covariance = (Eigen::MatrixXd::Identity(size, size) - K * H) * covariance;
}

void BII_9::updateStateWithGyroMeasurement(double gyroPitchRate, double gyroYawRate, double gyroRollRate) {
    state(6) += gyroPitchRate * dt;  // Update pitch
    state(7) += gyroYawRate * dt;    // Update yaw
    state(8) += gyroRollRate * dt;   // Update roll
}

void BII_9::estimateStateChange() {
    // Updating with GNSS Measurements
    updateWithGNSSMeasurements(Eigen::VectorXd& z);

    // Compute accelerometer measurements
    float accelInX = aclX.getAcceleration();
    float accelInY = aclY.getAcceleration();
    float accelInZ = aclZ.getAcceleration();

    // Updating with accelerometer measurements
    predictStep();
    updateStateWithAccelMeasurement(accelInX, accelInY, accelInZ);
    updateStep();
    updateCurrentVariables();

    // Update gyro measurements
    float gyroPitchRate = rlgPitch.getAngularVelocity();
    float gyroYawRate = rlgYaw.getAngularVelocity();
    float gyroRollRate = rlgRoll.getAngularVelocity();

    updateStateWithGyroMeasurement(gyroPitchRate, gyroYawRate, gyroRollRate);
}

void BII_9::estimateVelocityChange(double accelInX, double accelInY, double accelInZ) {
    predictStep();
    updateStateWithAccelMeasurement(accelInX, accelInY, accelInZ);
    updateStep();
    updateCurrentVariables();
    updateStateWithGyroMeasurement(float_t gyroPitchRate, float_t gyroYawRate, float_t gyroRollRate);
}

void BII_9::predictStep() {
    state = A * state;             // predict state
    covariance = A * covariance * A.transpose() + Q;  // predict covariance
}

void BII_9::updateStateWithAccelMeasurement(double accelInX, double accelInY, double accelInZ) {
    // Update velocity by integrating acceleration
    state(3) += accelInX * dt;  // Update velocity X
    state(4) += accelInY * dt;  // Update velocity Y
    state(5) += accelInZ * dt;  // Update velocity Z

    // Update position by integrating velocity
    state(0) += state(3) * dt;  // Update latitude (considering velocity in X-direction corresponds to latitude)
    state(1) += state(4) * dt;  // Update longitude (considering velocity in Y-direction corresponds to longitude
    state(2) += state(5) * dt;  // Update altitude (considering velocity in Z-direction corresponds to altitude)
}
void BII_9::updateStep() {
    Eigen::MatrixXd measurement(32, 1);  // measurement vector [latitude, longitude, ground speedX, accelInX, accelInY, accelInZ]
    measurement << currentLatitude, currentLongitude, currentGroundSpeed, accelInX, accelInY, accelInZ, Eigen::MatrixXd::Zero(26, 1);
    Eigen::MatrixXd innovation = measurement - H * state;  // Innovation
    Eigen::MatrixXd innovationCovariance = H * covariance * H.transpose() + R;  // Innovation covariance
    Eigen::MatrixXd kalmanGain = covariance * H.transpose() * innovationCovariance.inverse();  // Kalman gain

    // Update state and covariance
    state = state + kalmanGain * innovation;
    covariance = (Eigen::MatrixXd::Identity(32, 32) - kalmanGain * H) * covariance;
}

void BII_9::updateCurrentVariables() {
    // Update current latitude, longitude, and ground speedX with Kalman filter estimates
    currentLatitude = state(0, 0);
    currentLongitude = state(1, 0);
    currentGroundSpeed = state(2, 0);
}

void BII_9::estimatePositionChange() {

    // estimates change in position using accelerations and by updating states with Kalman Filter predictions
    // first estimates the velocity change using the accelerations in XYZ directions
    estimateVelocityChange(aclX.getAcceleration(), aclY.getAcceleration(), aclZ.getAcceleration());

    // perform Kalman Filter prediction step specifically for the position (latitude and longitude)
    // this step predicts the latitude and longitude by updating relevant block of the state matrix
    state.block(0, 0, 2, 1) = A.block(0, 0, 2, 2) * state.block(0, 0, 2, 1);

    // predict the covariance block corresponding to latitude and longitude; helps determine accuracy and uncertainty of estimate
    covariance.block(0, 0, 2, 2) = A.block(0, 0, 2, 2) * covariance.block(0, 0, 2, 2) * A.block(0, 0, 2, 2).transpose() + Q.block(0, 0, 2, 2);

    // Update current latitude and longitude based on ground speedX
    state(0, 0) += state(2, 0) * dt;  // Update latitude based on current ground speedX
    state(1, 0) += state(3, 0) * dt;  // Update longitude based on current ground speedX

}

// unit tests

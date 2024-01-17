//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "ringLaserGyroscopeV2.hpp"

RingLaserGyroscope::RingLaserGyroscope()
        : currentOrientation(0.0), previousAngularVelocity(0.0), previousOrientation(0.0) {}

double RingLaserGyroscope::updateOrientation(double angularVelocity, double deltaT) {
    double rawOrientation = previousOrientation + previousAngularVelocity * deltaT;
    double error = simulateError(-0.1f, 0.1f);
    currentOrientation = rawOrientation + error;
    previousAngularVelocity = angularVelocity;
    previousOrientation = currentOrientation;

    return currentOrientation;
}

double RingLaserGyroscope::simulateError(double minErr, double maxErr) {
    std::uniform_real_distribution<double> distribution(minErr, maxErr);
    double randomError = distribution(randomNumberGenerator);
    randomError += 0.01f * distribution(randomNumberGenerator);

    return randomError;
}

double RingLaserGyroscope::getAngularVelocity() const {
    return previousAngularVelocity;
}
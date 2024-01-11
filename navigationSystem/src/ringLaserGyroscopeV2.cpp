//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "ringLaserGyroscopeV2.hpp"

RingLaserGyroscope::RingLaserGyroscope()
        : currentOrientation(0.0), previousAngularVelocity(0.0), previousOrientation(0.0) {}

float_t RingLaserGyroscope::updateOrientation(float_t angularVelocity, float_t deltaT) {
    float_t rawOrientation = previousOrientation + previousAngularVelocity * deltaT;
    float_t error = simulateError(-0.1f, 0.1f);
    currentOrientation = rawOrientation + error;
    previousAngularVelocity = angularVelocity;
    previousOrientation = currentOrientation;

    return currentOrientation;
}

float_t RingLaserGyroscope::simulateError(float_t minErr, float_t maxErr) {
    std::uniform_real_distribution<float_t> distribution(minErr, maxErr);
    float_t randomError = distribution(randomNumberGenerator);
    randomError += 0.01f * distribution(randomNumberGenerator);

    return randomError;
}

float_t RingLaserGyroscope::getAngularRate() {
    return previousAngularVelocity;
}
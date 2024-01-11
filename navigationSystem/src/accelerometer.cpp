//
// Created by Tobias on 11/1/2024.
//

#include "accelerometer.hpp"

Accelerometer::Accelerometer() {
    measurementError = 0.1; // arbitrary value pending further research
}

// calculate single-axis linear acceleration
float Accelerometer::getAcceleration() {
    return previousVelocity = (currentVelocity - previousVelocity)/interval;
}
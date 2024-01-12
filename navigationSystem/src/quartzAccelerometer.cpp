//
// Created by Tobias on 11/1/2024.
//

#include "accelerometer.hpp"

QuartzAccelerometer::QuartzAccelerometer() {
    measurementError = 0.1; // arbitrary value pending further research
}

// calculate single-axis linear acceleration
float QuartzAccelerometer::getAcceleration() {
    return previousVelocity = (currentVelocity - previousVelocity)/interval;
}
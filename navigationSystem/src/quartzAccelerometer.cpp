//
// Created by Tobias on 11/1/2024.
//

#include "quartzAccelerometer.hpp"

QuartzAccelerometer::QuartzAccelerometer() {
    measurementError = 0.1; // arbitrary value pending further research
}

// calculate single-axis linear acceleration
double QuartzAccelerometer::getAcceleration() {
    return previousVelocity = (currentVelocity - previousVelocity)/interval;
}
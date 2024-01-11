//
// Created by Charles (Walrus) on 4/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX
#define SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX

#include <chrono>
#include <iostream>

/**
 * @class Accelerometer
 * @brief This class represents an accelerometer sensor.
 *
 * The Accelerometer class is used to calculate acceleration based on velocity and time interval.
 */
class Accelerometer {

public:
    Accelerometer();
    /**
     * @brief Get the acceleration calculated based on the previous and current velocity and the time interval.
     *
     * This function calculates the acceleration using the formula (currentVelocity - previousVelocity) / interval.
     * The acceleration value is a float.
     *
     * @return The calculated acceleration value.
     */
    float getAcceleration();

private:
    float currentVelocity;
    float previousVelocity;
    int interval;
    float measurementError;
};


#endif //SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX

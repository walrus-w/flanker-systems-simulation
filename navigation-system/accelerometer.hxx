//
// Created by Charles (Walrus) on 4/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX
#define SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX

#include <chrono>
#include <iostream>

class Accelerometer {

public:

    Accelerometer();
    static float calculateLinearAcceleration(float, float, float);  // parameters are:
                                                                        // final velocity in m/s
                                                                        // initial velocity in m/s
                                                                        // interval in milliseconds
private:

    float measurementError;

};

Accelerometer::Accelerometer() {
    measurementError = 0.1; // arbitrary value pending further research
}

// calculate single-axis linear acceleration
float Accelerometer::calculateLinearAcceleration(float previousState, float currentState, float interval) {
    return (currentState - previousState)/interval;
}

// unit test for calculateLinearAcceleration
int main() {
    float currentState = 50, previousState = 20, interval = 1000;
    float acceleration = Accelerometer::calculateLinearAcceleration(previousState, currentState, interval);
    std::cout << "The calculated linear acceleration is: " << acceleration << std::endl;
    return 0;
}

#endif //SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX

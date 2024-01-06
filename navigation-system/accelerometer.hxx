//
// Created by Charles (Walrus) on 4/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX
#define SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX

class Accelerometer {

public:
    Accelerometer();

private:

    float calculateDelta(float, float);

};

// calculate single-axis linear acceleration
float Accelerometer::calculateDelta(float lastState, float currentState) {
    return lastState + currentState;
}

#endif //SU_30_EFM_V2_7_3B_ACCELEROMETER_HXX

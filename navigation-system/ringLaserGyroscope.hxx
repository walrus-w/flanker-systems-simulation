//
// Created by Charles (Walrus) 03/01/2024
//

#include <iostream>
#include <cmath>
#include "blackBox.hxx"

class RingLaserGyroscope : public BlackBox {

public:
    // cold and dark constructor
    RingLaserGyroscope();

    // set angular velocity (rotation rate) of the gyroscope, degrees/second
    // velocity should probably be on the order of 75-100, as indicated for high-performance aircraft

private:
    // data members
    float_t angularVelocity = 95; // degrees/s - design angular velocity of the gyroscope,
                                  // estimated from public data which suggests aircraft LRGs typically operate at 70-100 degrees/s
                                  // higher angular velocity = greater accuracy
    float_t currentAngularVelocity; // present rate of rotation i.e. 0 when system is off and 95 when ready
    float_t interferenceFactor; // factor affecting interference pattern (e.g., due to rotation)
    float_t currentAngle;
    float_t correctionFactor;

    //double ringRadius;      // radius of the ring cavity -- currently unused
    //double laserWavelength = 632.8e-9; // wavelength of light emitted by Helium-Neon laser -- Currently unused

    // function members
    float_t generateRandomNoise(float_t min, float_t max);
    void applyCorrection();
    // takes current angle of pitch/roll/yaw from lua or EFM API and returns value for use by INS adjusted for error
    float_t calculateAngularChange(float_t rawInput);

};

// cold and dark constructor
RingLaserGyroscope::RingLaserGyroscope()
        : angularVelocity(0.0), interferenceFactor(0.0001), BlackBox() {}

// calculates generates value for random noise which is applied to interferenceFactor to simulate
float_t RingLaserGyroscope::generateRandomNoise(float_t min, float_t max) {
    return min + static_cast<double>(rand()) / RAND_MAX * (max - min);
}

// abstract implementation of software/hardware systems which increase accuracy
void RingLaserGyroscope::applyCorrection() {
    interferenceFactor *= correctionFactor;
}

// called by INS each time it updates (10 millisecond intervals), calculates a random calculation error via generateRandomNoise
// interferenceFactor is multiplied by error and rawInput (i.e. input angle of pitch/roll/yaw) is 'corrected' by adding the value of error
// calls applyCorrection, which
float_t RingLaserGyroscope::calculateAngularChange(float_t rawInput) {
    float_t error = generateRandomNoise(-0.1, 0.1);
    error *= interferenceFactor; // apply interference factor to error
    rawInput += error;
    interferenceFactor = std::cos(angularVelocity * 10); // update interference factor based on rotation
    applyCorrection(); // correct error;
    return rawInput;
}


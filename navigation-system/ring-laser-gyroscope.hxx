//
// Created by Charles (Walrus) 03/01/2024
//

/*
 * Basic implementation of gyroscope for measurement of positional change
 * This class takes the position of the aircraft from DCS ~60 times per second and calculates acceleration between each reference frame
 * Passes acceleration to instance of INS, which applies acceleration to velocity and position
 * It is intended for this class to be iteratively extended to become a reasonably accurate physical simulation of an LRG
 */

/*#ifndef SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX
#define SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX

#include <cstddef>
#include <cstdint>
#include <chrono>

using namespace std;

class RingLaserGyroscope {

public:

    RingLaserGyroscope();

private:
    // data members
    bool isReady;
    bool satAvailable;
    float forwardAcceleration; // acceleration measured in ms^-1
    float lateralAcceleration;
    float verticalAcceleration;

    // function members
    void initialisationSequence();

};

RingLaserGyroscope::RingLaserGyroscope() {
    isReady = false;
    forwardAcceleration = 0;
    lateralAcceleration = 0;
    verticalAcceleration = 0;
}

RingLaserGyroscope::initialisationSequence() {

}

#endif //SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX*/

#endif //SU_30_EFM_V2_7_3B_BLACK_BOX_HXX

#include <iostream>
#include <cmath>
#include "black-box.hxx"

class RingLaserGyroscope : public BlackBox {
private:
    double angularVelocity; // angular velocity of the gyroscope
    double ringRadius;      // radius of the ring cavity -- currently unused
    double laserWavelength = 632.8e-9; // wavelength of light emitted by Helium-Neon laser -- Currently unused
    double interferenceFactor; // factor affecting interference pattern (e.g., due to rotation)

public:
    // constructor
    RingLaserGyroscope(double radius, double wavelength);

    // set angular velocity (rotation rate) of the gyroscope, degrees/second
    // velocity should probably be on the order of 75-100
    void setAngularVelocity(double velocity);

    // simulate the gyroscope's behavior over a time period
    void simulate(double time); // Override the virtual function

    // additional member functions specific to RingLaserGyroscope

};

// constructor definition
RingLaserGyroscope::RingLaserGyroscope(double radius, double wavelength)
        : angularVelocity(0.0), ringRadius(radius), laserWavelength(wavelength), interferenceFactor(1.0) {}

// set angular velocity (rotation rate) definition
void RingLaserGyroscope::setAngularVelocity(double velocity) {
    angularVelocity = velocity;
}

// simulate the gyroscope's behavior over a time period definition
void RingLaserGyroscope::simulate(double time) {
    // update interference factor based on rotation
    interferenceFactor = std::cos(angularVelocity * time);

    // print simulated data or perform other actions based on the simulation
    std::cout << "Simulated Gyroscope Data:\n"
              << "Angular Velocity: " << angularVelocity << " rad/s\n"
              << "Interference Factor: " << interferenceFactor << "\n"
              << "Simulated Time: " << time << " seconds\n\n";
}

int main() {
    // create a RingLaserGyroscope object with specified radius and wavelength
    RingLaserGyroscope gyroscope(1.0, 632.8e-9); // Example wavelength: He-Ne laser

    // set angular velocity for simulation
    gyroscope.setAngularVelocity(0.1); // example angular velocity: 0.1 rad/s

    // simulate the gyroscope's behavior over a specified time period
    gyroscope.simulate(5.0); // example simulation time: 5 seconds

    return 0;
}

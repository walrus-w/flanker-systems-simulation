//
// Created by Charles (Walrus) 03/01/2024
//

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
    RingLaserGyroscope(double radius);

    // set angular velocity (rotation rate) of the gyroscope, degrees/second
    // velocity should probably be on the order of 75-100, as indicated for high-performance aircraft
    void setAngularVelocity(double velocity);

    // simulate the gyroscope's behavior over a time period
    void simulate(double time); // Override the virtual function

    // additional member functions specific to RingLaserGyroscope

};

// constructor definition
RingLaserGyroscope::RingLaserGyroscope(double radius)
        : angularVelocity(0.0), ringRadius(radius), interferenceFactor(1.0) {}

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
    RingLaserGyroscope gyroscope(1.0); // Example wavelength: He-Ne laser

    // set angular velocity for simulation
    gyroscope.setAngularVelocity(0.1); // example angular velocity: 0.1 rad/s

    // simulate the gyroscope's behavior over a specified time period
    gyroscope.simulate(5.0); // example simulation time: 5 seconds

    return 0;
}

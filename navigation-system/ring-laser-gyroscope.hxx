//
// Created by Charles (Walrus) 03/01/2024
//

/*
 * Basic implementation of gyroscope for measurement of positional change
 * This class takes the position of the aircraft from DCS ~60 times per second and calculates acceleration between each reference frame
 * Passes acceleration to instance of INS, which applies acceleration to velocity and position
 * It is intended for this class to be iteratively extended to become a reasonably accurate physical simulation of an LRG
 */

#ifndef SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX
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

#endif //SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX

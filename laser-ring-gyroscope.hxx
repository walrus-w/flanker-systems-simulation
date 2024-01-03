//
// Created by Charles (Walrus) 03/01/2024
//

/*
 * Basic implementation of gyroscope for measurement of positional change
 * It is intended for this class to be iteratively extended to become a reasonably accurate physical simulation of an LRG
 */

#ifndef SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX
#define SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX

#include <cstddef>
#include <cstdint>
#include <chrono>

using namespace std;

class LaserRingGyroscope {
public:
    LaserRingGyroscope()
private:

    //data members
    seconds alignmentTime;
    uint_least8_t currentSpeedError;    // initialised to an arbitrary maximum value, which decreases as alignment time increases
    uint_least8_t currentHeadingError;  // error will gradually accumulate post-alignment, unless corrected via satellite or (potentially) radar fix
    uint_least8_t currentNorthingError;
    uint_least8_t currentEastingError;

    // function members


};

#endif //SU_30_EFM_V2_7_3B_LASER_RING_GYROSCOPE_HXX

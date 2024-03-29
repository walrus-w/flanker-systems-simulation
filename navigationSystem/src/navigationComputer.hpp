//
// Created by Charles (Walrus) on 5/01/2024.
//
/*
 * interface between INS and cockpit displays
 * also takes input from pitot and static ports, returns CAS and baro altitude
 * this functionality may need to be emulated and these operations done in reverse (i.e. TAS converted to CAS)
 */

#ifndef SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX
#define SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX

#include <vector>
#include "blackBox.hpp"
#include "BII_9.hpp"


struct waypoint {
    float_t latitude;
    float_t longitude;
    float_t altitude;
};

struct radioNavBeacon : waypoint {
    double frequency;
};

struct ils : radioNavBeacon {
    waypoint outerMarker;
    waypoint middleMarker;
    waypoint innerMarker;
};

class NavigationComputer:BlackBox {
public:
    NavigationComputer();   // cold and dark constructor

    // interface functions
    int newWaypoint(float_t latitude, float_t longitude, float_t altitude);  // accepts waypoints as input from PS-5
    int setInstrumentLandingFrequency(float_t frequency);
    int setRadioNavFrequency(float_t frequency);   // set TACAN or RSBN frequency
                                                   // will interface with ILS script from A-29B mod
    void updateINS();
    // placeholder functions; declarations will need refactor depending upon availability of EFM API
    // if EFM API is unavailable, these functions will be passed data from lua environment
    int returnLatitude();
    int returnLongitude();
    int passInLatitude();
    int passInLongitude();
    int passInAccelerationX();
    int passInAccelerationY();
    int passInAccelerationZ();
    int passInAnglePitch();
    int passInAngleRoll();
    int passInAngleYaw();

private:
    BII_9 ins;
    std::vector<waypoint> waypointStorage;
    std::vector<radioNavBeacon> radioNavBeaconStorage;
    std::vector<ils> ilsStorage;
    const waypoint bullseye;

};


#endif //SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX

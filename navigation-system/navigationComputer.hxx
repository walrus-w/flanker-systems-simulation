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
#include "blackBox.hxx"
#include "BII_9.hxx"


struct waypoint {
    float_t latitude;
    float_t longitude;
    float_t altitude;
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
    InertialNavigationSystem ins;
    std::vector<waypoint> waypointStorage;
    const waypoint bullseye;

};

int NavigationComputer::returnLatitude() {
    return ins.getCurrentLat();
}

int NavigationComputer::returnLongitude() {
    return ins.getCurrentLong();
}


#endif //SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX

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

#include "black-box.hxx"
#include "inertial-navigation-system.hxx"

struct waypoint {
    float latitude;
    float longitude;
    float altitude;
};

class NavigationComputer:BlackBox {
public:
    void newWaypoint(float latitude, float longitude, float altitude);
private:
    InertialNavigationSystem ins;
    std::vector<waypoint> waypointStorage;



};

#endif //SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX

//
// Created by ctobi on 5/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX
#define SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX

#include "../black-box.hxx"
#include "inertial-navigation-system.hxx"

class NavigationComputer:BlackBox {
public:

private:
    InertialNavigationSystem ins;
    vector<float> waypointStorage;


};

#endif //SU_30_EFM_V2_7_3B_NAVIGATION_COMPUTER_HXX

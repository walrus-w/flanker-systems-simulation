//
// Created by Charles (Walrus) on 03/01/2024.
//

/*
 * This class is intended to act as a sort of data bus between the various systems which contribute to
 * navigation of the aircraft and to interface with cockpit instrumentation
 *
 */

#ifndef SU_30_EFM_V2_7_3B_NAVIGATION_SYSTEM_HXX
#define SU_30_EFM_V2_7_3B_NAVIGATION_SYSTEM_HXX

#include "inertial-navigation-system.hxx"
#include "air-data-computer.hxx"

class NavigationComplex {
public:

private:
    InertialNavigationSystem ins;
    AirDataComputer adc;

};


#endif //SU_30_EFM_V2_7_3B_NAVIGATION_SYSTEM_HXX

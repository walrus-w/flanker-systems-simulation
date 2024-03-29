//
// Created by Charles (Walrus) on 6/1/2024.
//

/*
 * initialises nav system independently of DCS, passes array of acceleration and time data to INS
 * checks position returned therefrom is correct
 */

#ifndef FLANKER_SYSTEMS_SIMULATION_NAV_SYSTEM_TEST_HXX
#define FLANKER_SYSTEMS_SIMULATION_NAV_SYSTEM_TEST_HXX

#include "BII_9.hpp"

class NavSystemTest {
public:

    NavSystemTest();

private:

    BII_9 testINS;

};

#endif //FLANKER_SYSTEMS_SIMULATION_NAV_SYSTEM_TEST_HXX

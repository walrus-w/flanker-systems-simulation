//
// Created by Charles (Walrus) on 03/01/2024.
//
/*
 * generic simulation of a modern digital Air Data Computer
 * accepts input from the INS and calculates positional and velocity data from this
 * returns data in format appropriate for cockpit display
 * in the absence of other information, should this implementation develop into a more complex simulation, it will be
 * based upon a TMS320-series processor, as these are known to be widely used by Russian aerospace manufacturers
 */


#ifndef SU_30_EFM_V2_7_3B_AIR_DATA_COMPUTER_HXX
#define SU_30_EFM_V2_7_3B_AIR_DATA_COMPUTER_HXX

#include "black-box.hxx"

class AirDataComputer:BlackBox {

public:

    // data members

    // function members
    AirDataComputer();

private:

    // data members


};

AirDataComputer::AirDataComputer() {

}

#endif //SU_30_EFM_V2_7_3B_AIR_DATA_COMPUTER_HXX

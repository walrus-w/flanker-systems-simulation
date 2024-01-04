//
// Created by Charles (Walrus) on 4/01/2024.
//
/*
 * Bas
 */

#ifndef SU_30_EFM_V2_7_3B_BLACK_BOX_HXX
#define SU_30_EFM_V2_7_3B_BLACK_BOX_HXX

#include "system-power-supply.hxx"

class BlackBox {

public:
    // data members

    // function members
    BlackBox();

private:
    // data members
    bool isOn;
    bool isReady;
    bool satelliteAntenna;
    PowerSupply powerSup(float maxWattage);

    // function members
    virtual void initialisationSequence();

};

BlackBox::BlackBox() {
    isOn = false;
    isReady = false;
}

#endif //SU_30_EFM_V2_7_3B_BLACK_BOX_HXX

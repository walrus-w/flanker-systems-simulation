//
// Created by Charles (Walrus) on 3/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX
#define SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX

class PowerSupply {

public:
    // data members

    // function members
    PowerSupply(float maximumCurrent)

private:
    // data members
    bool breakerClosed;
    float currentCurrent;
    float maxCurrent;

    // function members
};

PowerSupply::PowerSupply(float maximumCurrent) {
    breakerClosed = true;
    currentCurrent = 0;
    maxCurrent = maximumCurrent;
}

#endif //SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX

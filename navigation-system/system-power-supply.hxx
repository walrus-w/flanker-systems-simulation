//
// Created by Charles (Walrus) on 3/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX
#define SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX

class PowerSupply {

public:
    // data members

    // function members
    PowerSupply(int maximumVoltage);

private:
    // data members
    bool breakerClosed;
    float currentVoltage;
    float maxVoltage;

    // function members
};

PowerSupply::PowerSupply(int maximumVoltage) {
    breakerClosed = false;
    currentVoltage = 0;
    maxVoltage = maximumVoltage;
}

#endif //SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX

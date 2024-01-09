//
// Created by Charles (Walrus) on 3/01/2024.
//

#ifndef SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX
#define SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX

class PowerSupply {

public:
    // data members

    // function members
    PowerSupply(int maximumVoltage);    // cold and dark startup constructor
    PowerSupply(int maximumVoltage, float demand, bool circuit = true, bool powered = true);    // hot start constructor
    void closeBreaker();
    void openBreaker();

protected:
    // data members
    bool breakerClosed;     // circuit complete (i.e. power on) if true
    bool isSupplyingPower;  // distinct from breaker closed in that power may not be available even if circuit is complete
    float currentVoltage;
    float currentDemand;
    float maxVoltage;

    // function members

    virtual void onBreakerCloser();
};


// cold and dark start constructor
PowerSupply::PowerSupply(int maximumVoltage)
    : maxVoltage(maximumVoltage), breakerClosed(false), isSupplyingPower(false), currentDemand(0.0), currentVoltage(0.0) {}

// hot start constructor
PowerSupply::PowerSupply(int maximumVoltage, float demand, bool circuit, bool powered)
    : maxVoltage(maximumVoltage), currentDemand(demand), breakerClosed(circuit), isSupplyingPower(powered) {}

void PowerSupply::closeBreaker() {
    breakerClosed = true;
}

void PowerSupply::openBreaker() {
    breakerClosed = false;
}



#endif //SU_30_EFM_V2_7_3B_SYSTEM_POWER_SUPPLY_HXX

//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "systemPowerSupply.hpp"

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
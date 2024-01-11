//
// Created by Charles (Walrus) on 11/1/2024.
//

#include "navigationComputer.hpp"

int NavigationComputer::returnLatitude() {
    return ins.getCurrentLat();
}

int NavigationComputer::returnLongitude() {
    return ins.getCurrentLong();
}

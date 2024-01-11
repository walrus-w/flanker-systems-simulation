//
// Created by Charles (Walrus) on 5/01/2024.
//
/*
 * GNSS takes satellite position from DCS (i.e. actual unit position)
 * applies error derived from OEM documentation and returns position
 */

#include <iostream>

class GNSS {
public:
    float_t latitude;
    double longitude;
    double altitude;

    GNSS();

    // simulate GNSS update based on time
    void simulate(double time);
};



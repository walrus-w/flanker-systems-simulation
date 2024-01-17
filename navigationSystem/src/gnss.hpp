//
// Created by Charles (Walrus) on 5/01/2024.
//
/*
 * GNSS takes satellite position from DCS (i.e. actual unit position)
 * applies error derived from OEM documentation and returns position
 */

#include <iostream>

/**
 * @class GNSS
 * @brief This class is a simple simulation of a generic GNSS receiver
 *
 * A GNSS object will take position data and apply an error factor to it
 */

class GNSS {
public:
    // data members

    // function members
    GNSS();
    void setGNSS(double inputLatitude, double inputLongitude, double inputAltitude, double inputSpeedX,
                 double inputSpeedY, double inputHeading);
    [[nodiscard]] double getGNSSlatitude() const;
    [[nodiscard]] double getGNSSlongitude() const;
    [[nodiscard]] double getGNSSalt() const;
    [[nodiscard]] double getGNSSspeedX() const;
    [[nodiscard]] double getGNSSspeedY() const;
    [[nodiscard]] double getGNSSheading() const;

    // simulate GNSS update based on time
    void simulate(double time);

private:
    // data members
    double latitude;
    double longitude;
    double altitude;
    double speedX;
    double speedY;
    double heading;
};



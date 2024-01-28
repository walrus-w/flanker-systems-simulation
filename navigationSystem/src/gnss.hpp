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
    GNSS(double &newLatitude, double &newLongitude, double &newAltitude, double &newSpeedX, double &newSpeedY,
         double &newHeading, double &measurementError);
    void setGNSS(double inputLatitude, double inputLongitude, double inputAltitude, double inputSpeedX,
                 double inputSpeedY, double inputHeading);
    void updateGNSSlatitude(double newLat);
    [[nodiscard]] double getGNSSlatitude() const;
    void updateGNSSlongitude(double newLong);
    [[nodiscard]] double getGNSSlongitude() const;
    void updateGNSSalt(double newAlt);
    [[nodiscard]] double getGNSSalt() const;
    void updateGNSSspeedX(double newSpeed);
    [[nodiscard]] double getGNSSspeedX() const;
    void updateGNSSspeedY(double newSpeed);
    [[nodiscard]] double getGNSSspeedY() const;
    void updateGNSSHeading(double newHeading);
    [[nodiscard]] double getGNSSheading() const;


private:
    // data members
    double& latitude;
    double& longitude;
    double& altitude;
    double& speedX;
    double& speedY;
    double& heading;
    double& measurementError;

    // function members
    double& updateMeasurementError();
};



#ifndef SU_30_EFM_V2_7_3B_RINGLASERGYROSCOPEV2_HXX
#define SU_30_EFM_V2_7_3B_RINGLASERGYROSCOPEV2_HXX

#include <iostream>
#include <cmath>
#include <random>
#include "blackBox.hpp"

class RingLaserGyroscope : public BlackBox {
public:
    /**
     * @brief Constructs a new Ring Laser Gyroscope object with initial parameters.
     */
    RingLaserGyroscope();

    /**
       * @brief Updates the current orientation of the gyroscope using the current angular velocity,
       *        time interval, and previous orientation, then returns the new orientation.
       * @param angularVelocity The current angular velocity.
       * @param deltaT The time interval between function calls.
       * @return currentOrientation The updated orientation of the gyroscope.
       */
    float_t updateOrientation(float_t angularVelocity, float_t deltaT);

    /**
     * @brief Simulates the error in measurements due to hardware and software inaccuracies.
     * @param minErr The minimum error value.
     * @param maxErr The maximum error value.
     */
    float_t simulateError(float_t minErr, float_t maxErr);

    /**
     * @brief Returns the current Angular Rate.
     */
    float_t getAngularRate();

private:
    float_t currentOrientation;     ///< Current orientation of the gyroscope.
    float_t previousAngularVelocity;///< Previous angular velocity.
    float_t previousOrientation;    ///< Previous orientation.

    std::default_random_engine randomNumberGenerator;  ///< Generates random numbers for simulating measurement errors.
};



#endif  // SU_30_EFM_V2_7_3B_RINGLASERGYROSCOPEV2_HXX
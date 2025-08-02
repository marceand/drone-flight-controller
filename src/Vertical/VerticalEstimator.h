#pragma once

#include "../InertialSensor/InertialSensor.h"
#include "../Barometer/Barometer_BMP280.h"
#include "../KalmanFilter/AltitudeVelocityKF.h"
#include "../Attitude/AttitudeEstimator.h"

class VerticalEstimator
{
public:
    VerticalEstimator(Barometer_BMP280 &barometer,
                      AltitudeVelocityKF &altitudeVelocityKF,
                      InertialSensor &inertialSensor,
                      AttitudeEstimator &attitudeEstimator) : _barometer(barometer),
                                                              _altitudeVelocityKF(altitudeVelocityKF),
                                                              _inertialSensor(inertialSensor),
                                                              _attitudeEstimator(attitudeEstimator)

    {
    }

    void set_parameters();
    void update();
    float get_estimated_vertical_velocity()
    {
        return _estimated_vertical_velocity;
    }

private:
    Barometer_BMP280 &_barometer;
    AltitudeVelocityKF &_altitudeVelocityKF;
    InertialSensor &_inertialSensor;
    AttitudeEstimator &_attitudeEstimator;
    float _raw_vertical_acceleration = 0.0;
    float _estimated_vertical_velocity = 0.0;
    void calculateRawVerticalAcceleration();
    void estimateVerticalVelocity();
};

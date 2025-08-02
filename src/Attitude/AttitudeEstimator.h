#pragma once

#include "../InertialSensor/InertialSensor.h"
#include "../KalmanFilter/AngleKF.h"

class AttitudeEstimator
{
public:
    AttitudeEstimator(InertialSensor &inertialSensor,
                      AngleKF &rollKF,
                      AngleKF &pitchKF) : _inertialSensor(inertialSensor),
                                          _rollKF(rollKF),
                                          _pitchKF(pitchKF)
    {
    }

    void set_parameters();
    void update();

    float get_raw_roll()
    {
        return _raw_roll_angle;
    }

    float get_estimated_roll()
    {
        return _estimated_roll_angle;
    }
    float get_raw_pitch()
    {
        return _raw_pitch_angle;
    }

    float get_estimated_pitch()
    {
        return _estimated_pitch_angle;
    }

private:
    InertialSensor &_inertialSensor;
    AngleKF &_rollKF;
    AngleKF &_pitchKF;
    float _raw_roll_angle = 0.0f;
    float _raw_pitch_angle = 0.0f;
    float _estimated_roll_angle = 0.0f;
    float _estimated_pitch_angle = 0.0f;
};

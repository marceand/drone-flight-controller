
#pragma once

#include "AngleKalmanFilter.h"

class RollPitchAngleKF
{
public:
    void setRollParams(float dt, float processUncertainty, float measurementUncertainty);
    void setPitchParams(float dt, float processUncertainty, float measurementUncertainty);
    float calculateRoll(float angularRate, float angleMeasurement);
    float calculatePitch(float angularRate, float angleMeasurement);
private:
    AngleKalmanFilter _rollAngleKF;
    AngleKalmanFilter _pitchAngleKF;

};

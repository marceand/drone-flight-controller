
#pragma once

#include "AngleKalmanFilter.h"

class RollPitchAngleKF
{
public:
    void setParameters();
    float calculateRoll(float angularRate, float angleMeasurement);
    float calculatePitch(float angularRate, float angleMeasurement);

private:
    AngleKalmanFilter _rollAngleKF;
    AngleKalmanFilter _pitchAngleKF;
};

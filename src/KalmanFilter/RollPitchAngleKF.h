
#pragma once

#include "AngleKF.h"

class RollPitchAngleKF
{
public:
    void setParameters();
    float calculateRoll(float angularRate, float angleMeasurement);
    float calculatePitch(float angularRate, float angleMeasurement);

private:
    AngleKF _rollAngleKF;
    AngleKF _pitchAngleKF;
};

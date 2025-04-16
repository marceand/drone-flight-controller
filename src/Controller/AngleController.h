#pragma once

#include "../PID/CopterPID.h"


class AngleController
{
public:
    void setRollGains(float P, float I, float D);
    void setPitchGains(float P, float I, float D);
    void setTimeStep(float dt);
    void setOutputLimit(float limit);
    void setIntegralLimit(float limit);
    float computeRollPID(float desired, float actual);
    float computePitchPID(float desired, float actual);

private:
    CopterPID _rollPID;
    CopterPID _pitchPID;
};

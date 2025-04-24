#pragma once

#include "../PID/CopterPID.h"

class RateController
{

public:
    void setParameters();
    float computeRollPID(float desired, float actual);
    float computePitchPID(float desired, float actual);
    float computeYawPID(float desired, float actual);

private:
    CopterPID _rollPID;
    CopterPID _pitchPID;
    CopterPID _yawPID;
};

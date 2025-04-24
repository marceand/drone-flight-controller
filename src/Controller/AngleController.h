#pragma once

#include "../PID/CopterPID.h"

class AngleController
{
public:
    void setParameters();
    float computeRollPID(float desired, float actual);
    float computePitchPID(float desired, float actual);

private:
    CopterPID _rollPID;
    CopterPID _pitchPID;
};

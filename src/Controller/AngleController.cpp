#include "AngleController.h"

void AngleController::setRollGains(float P, float I, float D)
{
    _rollPID.setGains(P, I, D);
}

void AngleController::setPitchGains(float P, float I, float D)
{
    _pitchPID.setGains(P, I, D);
}

void AngleController::setTimeStep(float dt)
{
    _rollPID.setTimeStep(dt);
    _pitchPID.setTimeStep(dt);
}

void AngleController::setOutputLimit(float limit)
{
    _rollPID.setOutputLimit(limit);
    _pitchPID.setOutputLimit(limit);
}

void AngleController::setIntegralLimit(float limit)
{
    _rollPID.setIntegralLimit(limit);
    _pitchPID.setIntegralLimit(limit);
}

float AngleController::computeRollPID(float desired, float actual)
{
    return _rollPID.computePID(desired, actual);
}

float AngleController::computePitchPID(float desired, float actual)
{
    return _pitchPID.computePID(desired, actual);
}

#include "RateController.h"

void RateController::setRollGains(float P, float I, float D)
{
    _rollPID.setGains(P, I, D);
}

void RateController::setPitchGains(float P, float I, float D)
{
    _pitchPID.setGains(P, I, D);
}

void RateController::setYawGains(float P, float I, float D)
{
    _yawPID.setGains(P, I, D);
}

void RateController::setTimeStep(float dt)
{
    _rollPID.setTimeStep(dt);
    _pitchPID.setTimeStep(dt);
    _yawPID.setTimeStep(dt);
}

void RateController::setOutputLimit(float limit)
{
    _rollPID.setOutputLimit(limit);
    _pitchPID.setOutputLimit(limit);
    _yawPID.setOutputLimit(limit);
}

void RateController::setIntegralLimit(float limit)
{
    _rollPID.setIntegralLimit(limit);
    _pitchPID.setIntegralLimit(limit);
    _yawPID.setIntegralLimit(limit);
}

float RateController::computeRollPID(float desired, float actual)
{
    return _rollPID.computePID(desired, actual);
}

float RateController::computePitchPID(float desired, float actual)
{
    return _pitchPID.computePID(desired, actual);
}

float RateController::computeYawPID(float desired, float actual)
{
    return _yawPID.computePID(desired, actual);
}

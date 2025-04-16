#include "CopterPID.h"

void CopterPID::setGains(float P, float I, float D)
{
    _kP = P;
    _kI = I;
    _kD = D;
}

void CopterPID::setTimeStep(float dt)
{
    _dt = dt;
}

float CopterPID::computePID(float desired, float actual)
{
    float error = desired - actual;
    float output = computeProportional(error) + computeIntegral(error) + computerDerivative(error);
    _lastError = error;

    return constrainOutput(output, -_limitOutput, _limitOutput);
}

float CopterPID::computeProportional(float error)
{
    return _kP * error;
}

float CopterPID::computeIntegral(float error)
{
    float newIntegral = _lastIntegral + _kI * (error + _lastError) * _dt / 2;
    return constrainOutput(newIntegral, -_limitIntegral, _limitIntegral);
}

float CopterPID::computerDerivative(float error)
{
    return _kD * (error - _lastError) / _dt;
}

float CopterPID::constrainOutput(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

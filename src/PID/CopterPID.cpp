#include "CopterPID.h"

void CopterPID::setParameters(float P, float I, float D, float dt, float outputLimit, float integralLimit)
{
    setGains(P, I, D);
    setTimeStep(dt);
    setOutputLimit(outputLimit);
    setIntegralLimit(integralLimit);
}

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

void CopterPID::setOutputLimit(float limit)
{
    _limitOutput = limit;
};
void CopterPID::setIntegralLimit(float limit)
{
    _limitIntegral = limit;
}
void CopterPID::resetIntegral()
{
    _lastIntegral = 0.f;
};

float CopterPID::computePID(float desired, float actual, bool integrator_enabled)
{
    float error = desired - actual;
    float output = computeProportional(error) + computeIntegral(error, integrator_enabled) + computerDerivative(error);
    _lastError = error;

    return constrainOutput(output, -_limitOutput, _limitOutput);
}

float CopterPID::computeProportional(float error)
{
    return _kP * error;
}

float CopterPID::computeIntegral(float error, bool integrator_enabled)
{
    if (!integrator_enabled)
    {
        _lastIntegral = 0.f;
        return 0.f;
    }

    float newIntegral = _lastIntegral + _kI * (error + _lastError) * _dt / 2.0f;
    _lastIntegral = constrainOutput(newIntegral, -_limitIntegral, _limitIntegral);
    return _lastIntegral;
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

#pragma once

class CopterPID
{

public:
    void setGains(float P, float I, float D);
    void setTimeStep(float dt);
    void setOutputLimit(float limit) { _limitOutput = limit; };
    void setIntegralLimit(float limit) { _limitIntegral = limit; };
    float computePID(float desired, float actual);

private:
    float _dt{1.f};
    float _kP{0.f};
    float _kI{0.f};
    float _kD{0.f};
    float _lastError{0.f};
    float _lastIntegral{0.f};
    float _limitOutput{0.f};
    float _limitIntegral{0.f};

    float computeProportional(float error);
    float computeIntegral(float error);
    float computerDerivative(float error);
    float constrainOutput(float value, float minValue, float maxValue);
};
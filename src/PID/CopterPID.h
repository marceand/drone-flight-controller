#pragma once

class CopterPID
{

public:
    void setParameters(float P, float I, float D, float dt, float outputLimit, float integralLimit);
    void setGains(float P, float I, float D);
    void setTimeStep(float dt);
    void setOutputLimit(float limit);
    void setIntegralLimit(float limit);
    void reset();
    void set_integrator(bool enable);
    float computePID(float desired, float measured);

private:
    float _dt{1.f};
    float _kP{0.f};
    float _kI{0.f};
    float _kD{0.f};
    float _lastError{0.f};
    float _lastIntegral{0.f};
    float _limitOutput{0.f};
    float _limitIntegral{0.f};
    bool _integrator_enabled = false;
    float computeProportional(float error);
    float computeIntegral(float error, bool integrator_enabled);
    float computerDerivative(float error);
    float constrainOutput(float value, float minValue, float maxValue);
};
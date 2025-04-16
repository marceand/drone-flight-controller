#include "RollPitchAngleKF.h"

void RollPitchAngleKF::setRollParams(float dt, float processUncertainty, float measurementUncertainty)
{
    _rollAngleKF.setParams(dt, processUncertainty, measurementUncertainty);
}

void RollPitchAngleKF::setPitchParams(float dt, float processUncertainty, float measurementUncertainty)
{
    _pitchAngleKF.setParams(dt, processUncertainty, measurementUncertainty);
}

float RollPitchAngleKF::calculateRoll(float angularRate, float angleMeasurement)
{
    return _rollAngleKF.calculate(angularRate, angleMeasurement);
}

float RollPitchAngleKF::calculatePitch(float angularRate, float angleMeasurement)
{
    return _pitchAngleKF.calculate(angularRate, angleMeasurement);
}

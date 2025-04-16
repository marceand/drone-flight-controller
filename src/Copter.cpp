#include "Copter.h"

void Copter::init(void)
{

    _inertialSensor.init();

    _rateController.setParams();
    _angleController.setParams();
    _rollPitchAngleKF.setParams();

    // _rateController.setRollGains(float P, float I, float D);
    // _rateController.setPitchGains(float P, float I, float D);
    // _rateController.setYawGains(float P, float I, float D);
    // _rateController.setTimeStep(float dt);
    // _rateController.setOutputLimit(float limit);
    // _rateController.setIntegralLimit(float limit);

    // _angleController.setRollGains(float P, float I, float D);
    // _angleController.setPitchGains(float P, float I, float D);
    // _angleController.setTimeStep(float dt);
    // _angleController.setOutputLimit(float limit);
    // _angleController.setIntegralLimit(float limit);

    // _rollPitchAngleKF.setPitchParams();
    // _rollPitchAngleKF.setRollParams();

    _rc.init();
}

void Copter::run(void)
{
    _rc.read();
    float desiredRollAngle = _rc.getDesiredRollAngle();
    float desiredPitchAngle = _rc.getDesiredPitchAngle();
    float desiredYawRate = _rc.getDesiredYawRate();
    float throttleInput = _rc.getThrottleInPWM();

    _inertialSensor.read();

    float rollRate = _inertialSensor.getCalibGyroX();
    float pitchRate = _inertialSensor.getCalibGyroY();
    float yawRate = _inertialSensor.getCalibGyroZ();
    float rollAngle = _inertialSensor.getRollAngle();
    float pitchAngle = _inertialSensor.getPitchAngle();

    float rollAngleKF = _rollPitchAngleKF.calculateRoll(rollRate, rollAngle);
    float pitchAngleKF = _rollPitchAngleKF.calculatePitch(pitchRate, pitchAngle);

    float desiredRollRate = _angleController.computeRollPID(desiredRollAngle, rollAngleKF);
    float desiredPitchRate = _angleController.computePitchPID(desiredPitchAngle, pitchAngleKF);

    float rollInput = _rateController.computeRollPID(desiredRollRate, rollRate);
    float pitchInput = _rateController.computePitchPID(desiredPitchRate, pitchRate);
    float yawInput = _rateController.computeYawPID(desiredYawRate, yawRate);
}

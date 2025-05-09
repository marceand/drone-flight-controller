#include "Copter.h"

void Copter::init(void)
{
    _battMonitor.init();
    // _inertialSensor.init();

    // _rateController.setParameters();
    // _angleController.setParameters();
    // _rollPitchAngleKF.setParameters();

    // _rc.init();
}

void Copter::run(void)
{
    // _rc.read();
    // float desiredRollAngle = _rc.getDesiredRollAngle();
    // float desiredPitchAngle = _rc.getDesiredPitchAngle();
    // float desiredYawRate = _rc.getDesiredYawRate();
    // float throttleInput = _rc.getThrottleInPWM();

    // _inertialSensor.read();

    // float rollRate = _inertialSensor.getCalibGyroX();
    // float pitchRate = _inertialSensor.getCalibGyroY();
    // float yawRate = _inertialSensor.getCalibGyroZ();
    // float rollAngle = _inertialSensor.getRollAngle();
    // float pitchAngle = _inertialSensor.getPitchAngle();

    // float rollAngleKF = _rollPitchAngleKF.calculateRoll(rollRate, rollAngle);
    // float pitchAngleKF = _rollPitchAngleKF.calculatePitch(pitchRate, pitchAngle);

    // float desiredRollRate = _angleController.computeRollPID(desiredRollAngle, rollAngleKF);
    // float desiredPitchRate = _angleController.computePitchPID(desiredPitchAngle, pitchAngleKF);

    // float rollInput = _rateController.computeRollPID(desiredRollRate, rollRate);
    // float pitchInput = _rateController.computePitchPID(desiredPitchRate, pitchRate);
    // float yawInput = _rateController.computeYawPID(desiredYawRate, yawRate);
}

void Copter::check_esc_calibration()
{
    // read the radio until get input
    _rc.read();

    // check for calibration mode
    // check for maxium throthtle
    // block until we restart
    // while (1)
    // {
    //     hal.scheduler->delay(5);
    // }

    float throttle = _rc.getThrottleInPWM();
}

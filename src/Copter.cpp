#include "Copter.h"

#define ESC_CALIBRATION_HIGH_THROTTLE 1800

void Copter::init(void)
{
    _ledIndicator.init();
    _storage.init();
    _motors.init();
    _rc.init();
    //_battMonitor.init();
    // _inertialSensor.init();

    // _rateController.setParameters();
    // _angleController.setParameters();
    // _rollPitchAngleKF.setParameters();

    check_esc_calibration();
}

void Copter::run(void)
{
    _rc.read();
    float desiredRollAngle = _rc.getDesiredRollAngle();
    float desiredPitchAngle = _rc.getDesiredPitchAngle();
    float desiredYawRate = _rc.getDesiredYawRate();
    float throttleInput = _rc.getThrottleInPWM();

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
    _ledIndicator.enableRedLED();

    uint8_t i = 0;
    while (i++ < 100)
    {
        delay(20);
        _rc.read();
    }

    _ledIndicator.disableRedLED();

    if (_storage.check_for_esc_calibration())
    {

        if (_rc.getThrottleInPWM() >= ESC_CALIBRATION_HIGH_THROTTLE)
        {
            _ledIndicator.enableGreenLED();
            _storage.set_check_esc_calibration(false);
            while (1)
            {
                delay(5);
            }
        }
    }
    else
    {
        if (_rc.getThrottleInPWM() >= ESC_CALIBRATION_HIGH_THROTTLE)
        {
            _storage.set_check_esc_calibration(true);
            _motors.setArm(true);
            _ledIndicator.enableGreenLED();
            _ledIndicator.enableRedLED();

            while (1)
            {
                _rc.read();
                delay(10);
                float throttleInput = _rc.getThrottleInPWM();
                _motors.calibrateESC(throttleInput);
            }
        }
    }
}

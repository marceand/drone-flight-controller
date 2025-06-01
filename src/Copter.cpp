#include "Copter.h"

#define ESC_CALIBRATION_HIGH_THROTTLE 1800

void Copter::init(void)
{
    _storage.init();
    _inertialSensor.init();
    _barometer.init();
    _rc.init();
    _ledIndicator.init();
    _motors.init();
    _battMonitor.init();
    _rateRollController.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    _ratePitchController.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    _rateYawController.setParameters(2, 12, 0, 0.004, 400, 400);
    _angleRollController.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    _anglePitchController.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    _velocityController.setParameters(3.5, 0.0015, 0.01, 0.004, 400, 400);

    check_esc_calibration();
}

void Copter::run(void)
{
    _inertialSensor.read();
    float rollRate = _inertialSensor.getCalibGyroX();
    float pitchRate = _inertialSensor.getCalibGyroY();
    float yawRate = _inertialSensor.getCalibGyroZ();
    float rollAngle = _inertialSensor.getRollAngle();
    float pitchAngle = _inertialSensor.getPitchAngle();
    float verticalAcceleration = _inertialSensor.getVerticalAcceleration();

    _barometer.read();
    float relativeAltitude = _barometer.get_relative_altitude_in_cm();

    _rc.read();
    float desiredRollAngle = _rc.getDesiredRollAngle();
    float desiredPitchAngle = _rc.getDesiredPitchAngle();
    float desiredYawRate = _rc.getDesiredYawRate();
    float desiredThrottleVelocity = _rc.getDesiredThrottleVelocity();

    float rollAngleKF = _rollKF.calculateAngle(rollRate, rollAngle);
    float pitchAngleKF = _pitchKF.calculateAngle(pitchRate, pitchAngle);
    float verticalVelocityKF = _altitudeVelocityKF.calculateVerticalVelocity(relativeAltitude, verticalAcceleration);

    float desiredRollRate = _angleRollController.computePID(desiredRollAngle, rollAngleKF);
    float desiredPitchRate = _anglePitchController.computePID(desiredPitchAngle, pitchAngleKF);

    float rollInput = _rateRollController.computePID(desiredRollRate, rollRate);
    float pitchInput = _ratePitchController.computePID(desiredPitchRate, pitchRate);
    float yawInput = _rateYawController.computePID(desiredYawRate, yawRate);
    float throttleInput = _velocityController.computePID(desiredThrottleVelocity, verticalVelocityKF);
    float throttleHoverInput = _rc.getMidThrottle() + throttleInput;

    _motors.runMotors(throttleHoverInput, rollInput, pitchInput, yawInput);
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

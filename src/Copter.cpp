#include "Copter.h"

#define ESC_CALIBRATION_HIGH_THROTTLE 1800
#define MOTORS_MINIMUM_STARTUP_THROTTLE 1015
#define HZ_TO_US(hz) (1000000UL / (hz))

Copter::Task Copter::tasks[] = {
    // {"TaskA", HZ_TO_US(250), 0, run_main_controller},
    // {"TaskA", HZ_TO_US(100), 0, taskA},
    // {"TaskB", HZ_TO_US(50), 0, taskB},
};

const int Copter::NUM_TASKS = sizeof(tasks) / sizeof(Task);

void Copter::init(void)
{
    _storage.init();
    _inertialSensor.init();
    _barometer.init();
    _rc.init();
    _ledIndicator.init();
    _motors.init();
    // _battMonitor.init();
    // _rateRollController.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    // _ratePitchController.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    // _rateYawController.setParameters(2, 12, 0, 0.004, 400, 400);
    // _angleRollController.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchController.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _velocityController.setParameters(3.5, 0.0015, 0.01, 0.004, 400, 400);

    check_esc_calibration();
    check_motors_startup();
    check_motors_mapping();
    arm_esc_at_minimum();
}

void Copter::run(void)
{
    // uint32_t now = micros();

    // for (int i = 0; i < NUM_TASKS; i++)
    // {
    //     Task &task = tasks[i];
    //     if (now - task.last_run_us >= task.interval_us)
    //     {
    //         task.func();
    //         task.last_run_us = now;
    //     }
    // }
}

void Copter::run_main_controller()
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
            // Keep in the loop until drone is reboot
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
                float throttleInput = _rc.getThrottleInPWM();
                _motors.runMotorsForESCPassthrough(throttleInput);
                delay(4);
            }
        }
    }
}

void Copter::check_motors_startup()
{
    _ledIndicator.enableRedLED();
    _ledIndicator.enableGreenLED();

    arm_esc_at_minimum();

    _motors.setArm(true);

    uint32_t start_ms = millis();
    uint32_t test_interval_ms = 10000;
    while (millis() - start_ms < test_interval_ms)
    {
        _motors.runMotorsForESCPassthrough(MOTORS_MINIMUM_STARTUP_THROTTLE);
        delay(4);
    }

    arm_esc_at_minimum();

    _ledIndicator.disableRedLED();
    _ledIndicator.disableGreenLED();
}

void Copter::check_motors_mapping()
{
    _ledIndicator.enableRedLED();
    _ledIndicator.enableGreenLED();

    arm_esc_at_minimum();

    _motors.setArm(true);

    uint32_t test_interval_ms = 5000;
    uint32_t total_ms_per_motor = 2 * test_interval_ms;
    int num_of_motors = 4;

    for (int motor_sequence = 1; motor_sequence <= num_of_motors; motor_sequence++)
    {
        uint32_t start_ms = millis();
        uint32_t now_ms = start_ms;
        while (now_ms - start_ms < total_ms_per_motor)
        {
            if (now_ms - start_ms < test_interval_ms)
            {
                _motors.runMotorInSequence(motor_sequence, MOTORS_MINIMUM_STARTUP_THROTTLE);
            }
            else
            {
                _motors.runAtMinimum();
            }
            delay(4);
            now_ms = millis();
        }
    }

    _motors.setArm(false);

    arm_esc_at_minimum();

    _ledIndicator.disableRedLED();
    _ledIndicator.disableGreenLED();
}

void Copter::arm_esc_at_minimum()
{
    _motors.setArm(true);

    uint32_t start_ms = millis();
    uint32_t test_interval_ms = 3000;
    while (millis() - start_ms < test_interval_ms)
    {
        _motors.runAtMinimum();
        delay(4);
    }

    _motors.setArm(false);
}

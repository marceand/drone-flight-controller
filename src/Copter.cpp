#include "Copter.h"
#include <Wire.h>

#define ESC_CALIBRATION_HIGH_THROTTLE 1800
#define MOTORS_MINIMUM_STARTUP_THROTTLE 1015
#define ARM_DELAY 20    // called at 10hz so 2 seconds
#define DISARM_DELAY 20 // called at 10hz so 2 seconds
#define HZ_TO_US(hz) (1000000UL / (hz))

// Copter::Task Copter::tasks[] = {
//     // {"TaskA", HZ_TO_US(250), 0, run_main_controller},
//     // {"TaskA", HZ_TO_US(100), 0, taskA},
//     // {"TaskB", HZ_TO_US(50), 0, taskB},
// };

// const int Copter::NUM_TASKS = sizeof(tasks) / sizeof(Task);

void Copter::init(void)
{
    _storage.init();
    _inertialSensor.init();
    _barometer.init();
    _rc.init();
    _ledIndicator.init();
    _motors.init();
    //_battMonitor.init();
    _rollKF.setParameters();
    _pitchKF.setParameters();
    _altitudeVelocityKF.setParameters();
    _rateRollController.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    _ratePitchController.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    _rateYawController.setParameters(2, 12, 0, 0.004, 400, 400);
    _angleRollController.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    _anglePitchController.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    _velocityController.setParameters(3.5, 0.0015, 0.01, 0.004, 400, 400);

    // check_esc_calibration();
    // check_motors_startup();
    // check_motors_mapping();
    // arm_esc_at_minimum();
}

void Copter::run(void)
{
    read_rc_channels();
    read_inertial_sensor();
    read_barometer();
    check_takeoff();
    run_main_controller();
    // check_motors_arming();

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

void Copter::check_takeoff()
{
    if (!is_flying && _rc.getThrottleInPWM() > 1400)
    {
        is_flying = true;
    }
    if (_rc.getThrottleInPWM() < 1050)
    {
        is_flying = false;
        _angleRollController.reset();
        _anglePitchController.reset();
        _rateRollController.reset();
        _ratePitchController.reset();
        _rateYawController.reset();
        _velocityController.reset();
    }
}

void Copter::read_rc_channels()
{
    _rc.read();
    _motors.set_throttle_radio(_rc.getThrottleInPWM());
}

void Copter::read_inertial_sensor()
{
    _inertialSensor.read();
}

void Copter::read_barometer()
{
    _barometer.read();
}

void Copter::run_main_controller()
{
    float rollRate = _inertialSensor.getCalibGyroX();
    float pitchRate = _inertialSensor.getCalibGyroY();
    float yawRate = _inertialSensor.getCalibGyroZ();
    float rollAngle = _inertialSensor.getRollAngle();
    float pitchAngle = _inertialSensor.getPitchAngle();
    float verticalAcceleration = _inertialSensor.getVerticalAcceleration();

    float relativeAltitude = _barometer.get_relative_altitude_in_cm();

    float desiredRollAngle = _rc.getDesiredRollAngle();
    float desiredPitchAngle = _rc.getDesiredPitchAngle();
    float desiredYawRate = _rc.getDesiredYawRate();
    float desiredThrottleVelocity = _rc.getDesiredThrottleVelocity();

    float rollAngleKF = _rollKF.calculateAngle(rollRate, rollAngle);
    float pitchAngleKF = _pitchKF.calculateAngle(pitchRate, pitchAngle);
    float verticalVelocityKF = _altitudeVelocityKF.calculateVerticalVelocity(relativeAltitude, verticalAcceleration);

    float desiredRollRate = _angleRollController.computePID(desiredRollAngle, rollAngleKF, is_flying);
    float desiredPitchRate = _anglePitchController.computePID(desiredPitchAngle, pitchAngleKF, is_flying);

    float rollCommand = _rateRollController.computePID(desiredRollRate, rollRate, is_flying);
    float pitchCommand = _ratePitchController.computePID(desiredPitchRate, pitchRate, is_flying);
    float yawCommand = _rateYawController.computePID(desiredYawRate, yawRate, is_flying);
    float throttleHover = _velocityController.computePID(desiredThrottleVelocity, verticalVelocityKF, is_flying);
    float throttleCommand = _rc.getMidThrottle() + throttleHover;

    _motors.set_command_inputs(throttleCommand, rollCommand, pitchCommand, yawCommand);
}

void Copter::run_motors()
{
    _motors.update_outputs();
    _motors.write_to_motors();
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
                _motors.set_esc_calibration_throttle(throttleInput);
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
        _motors.set_esc_calibration_throttle(MOTORS_MINIMUM_STARTUP_THROTTLE);
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
                _motors.set_motor_sequence_throttle(motor_sequence, MOTORS_MINIMUM_STARTUP_THROTTLE);
            }
            else
            {
                _motors.set_motor_stop_throttle();
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
        _motors.set_motor_stop_throttle();
        delay(4);
    }

    _motors.setArm(false);
}

void Copter::check_motors_arming()
{
    if (_rc.getThrottleInPWM() > 1005)
    {
        _arming_counter = 0;
        return;
    }

    uint16_t yaw_in_pwm = _rc.getYawInPWM();
    if (yaw_in_pwm >= 1995)
    {
        if (_arming_counter < ARM_DELAY)
        {
            _arming_counter++;
        }

        if (_arming_counter == ARM_DELAY && !_motors.isArmed())
        {
            _ledIndicator.enableGreenLED();
            _motors.setArm(true);
            _motors.set_motor_stop_throttle();
        }
    }
    else if (yaw_in_pwm <= 1005)
    {
        if (_arming_counter <= DISARM_DELAY)
        {
            _arming_counter++;
        }

        if (_arming_counter == DISARM_DELAY && _motors.isArmed())
        {
            _ledIndicator.disableGreenLED();
            _motors.set_motor_stop_throttle();
            _motors.setArm(false);
        }
    }
    else
    {
        _arming_counter = 0;
    }
}

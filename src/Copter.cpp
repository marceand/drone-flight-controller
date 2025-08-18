#include "Copter.h"
#include <Wire.h>

#define ESC_CALIBRATION_HIGH_THROTTLE 1800
#define MOTORS_MINIMUM_STARTUP_THROTTLE 1015
#define ARM_DELAY 20    // called at 10hz so 2 seconds
#define DISARM_DELAY 20 // called at 10hz so 2 seconds

void Copter::init(void)
{
    _toneAlarm.init();
    ToneAlarm::events.startup = true;

    uint32_t start_ms = millis();
    uint32_t now_ms = start_ms;
    while (now_ms - start_ms < 5000)
    {
        _toneAlarm.update();
        delay(20);
        now_ms = millis();
    }

    // _storage.init();
    // _inertialSensor.init();
    // _barometer.init();
    // _rc.init();
    // _ledIndicator.init();
    // _motors.init();
    // _attitudeEstimator.set_parameters();
    // _attitudeController.set_parameters();
    // _verticalEstimator.set_parameters();
    // _verticalVelocityController.set_parameters();
    //_battMonitor.init();

    // check_esc_calibration();
    // check_motors_startup();
    // check_motors_mapping();
    // arm_esc_at_minimum();
}

void Copter::read_rc_channels()
{
    _rc.read();
    _attitudeController.set_desired_angles(_rc.getDesiredRollAngle(), _rc.getDesiredPitchAngle());
    _attitudeController.set_desired_yaw_rate(_rc.getDesiredYawRate());
    _verticalVelocityController.set_desired_vertical_velocity(_rc.getDesiredThrottleVelocity());
    _motors.set_throttle_radio(_rc.getThrottleInPWM());
    Serial.print("Time:");
    Serial.print(micros());
    Serial.print("\t");
    Serial.print("Throtle:");
    Serial.print(_rc.getThrottleInPWM());
    Serial.print("\t");
}

void Copter::read_inertial_sensor()
{
    _inertialSensor.read();
    _attitudeController.set_measured_rates(_inertialSensor.getCalibGyroX(),
                                           _inertialSensor.getCalibGyroY(),
                                           _inertialSensor.getCalibGyroZ());
    Serial.print("AccX:");
    Serial.print(_inertialSensor.getCalibAccelX());
    Serial.print("\t");
    Serial.print("AccY:");
    Serial.print(_inertialSensor.getCalibAccelY());
    Serial.print("\t");
    Serial.print("AccZ:");
    Serial.print(_inertialSensor.getCalibAccelZ());
    Serial.print("\t");
}

void Copter::read_barometer()
{
    _barometer.read();
}

void Copter::check_takeoff()
{
    if (!is_flying && _rc.getThrottleInPWM() > 1400)
    {
        is_flying = true;
        _attitudeController.set_integrator(is_flying);
        _verticalVelocityController.set_integrator(is_flying);
    }
    if (_rc.getThrottleInPWM() < 1050)
    {
        is_flying = false;
        _attitudeController.set_integrator(is_flying);
        _verticalVelocityController.set_integrator(is_flying);
        _attitudeController.reset();
        _verticalVelocityController.reset();
    }
}

void Copter::run_main_controller()
{
    _attitudeEstimator.update();
    _verticalEstimator.update();

    _attitudeController.update(_attitudeEstimator.get_estimated_roll(), _attitudeEstimator.get_estimated_pitch());
    _verticalVelocityController.update(_verticalEstimator.get_estimated_vertical_velocity());

    float roll_command = _attitudeController.get_roll_command();
    float pitch_command = _attitudeController.get_pitch_command();
    float yaw_command = _attitudeController.get_yaw_command();
    float hover_command = _verticalVelocityController.get_hover_command();
    float throttle_command = _rc.getMidThrottle() + hover_command;

    // Serial.print("Throttle-Command:");
    // Serial.print(throttle_command);
    // Serial.print("\t");
    // Serial.print("Roll-Command:");
    // Serial.print(roll_command);
    // Serial.print("\t");
    // Serial.print("Pitch-Command:");
    // Serial.print(pitch_command);
    // Serial.print("\t");
    // Serial.print("Yaw-Command:");
    // Serial.print(yaw_command);
    // Serial.println();
    _motors.set_command_inputs(throttle_command, roll_command, pitch_command, yaw_command);
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
        _motors.write_to_motors();
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
            _motors.write_to_motors();
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
        _motors.write_to_motors();
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

void Copter::run_tone_alarm()
{
    _toneAlarm.update();
}

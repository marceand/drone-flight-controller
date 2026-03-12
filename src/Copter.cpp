#include "Copter.h"
#include <Wire.h>

#define ESC_CALIBRATION_HIGH_THROTTLE 1800
#define MOTORS_MINIMUM_STARTUP_THROTTLE 1016
#define ARM_DELAY 20    // called at 10hz so 2 seconds
#define DISARM_DELAY 20 // called at 10hz so 2 seconds

void Copter::init(void)
{
    _notifier.init();
    _storage.init();
    _rc.init();
    _motors.init();
    check_esc_calibration(); // need to call after power on otherwise ESCs calibration time window is missed
    _inertialSensor.init();
    _barometer.init();
    _logger.init(_storage.get_session_id());
    _attitudeEstimator.set_parameters();
    _attitudeController.set_parameters();
    _verticalEstimator.set_parameters();
    _verticalVelocityController.set_parameters();
    _battMonitor.init();
    _logger.insert_parameters_to_buffer();

    check_motors_startup();
    // check_motors_mapping();
    arm_esc_at_minimum();
}

void Copter::read_rc_channels()
{
    _rc.read();
    _attitudeController.set_desired_angles(_rc.get_desired_roll_angle(), _rc.get_desired_pitch_angle());
    _attitudeController.set_desired_rates(_rc.get_desired_roll_rate(), _rc.get_desired_pitch_rate(), _rc.get_desired_yaw_rate());
    _verticalVelocityController.set_desired_vertical_velocity(_rc.get_desired_vertical_velocity());
    _motors.set_throttle_radio(_rc.get_throttle_in_pwm());
    _motors.set_motor_emergency(_rc.is_motor_emergency());

    if (_rc.is_radio_failsafe() && _motors.is_armed())
    {
        _motors.setArm(false);
        _logger.update_radio_failsafe(true);
    }

    _logger.update_desired_angles(_rc.get_desired_roll_angle(),
                                  _rc.get_desired_pitch_angle());
    _logger.update_desired_rates(_rc.get_desired_roll_rate(), _rc.get_desired_pitch_rate(), _rc.get_desired_yaw_rate());
    _logger.update_desired_vertical_velocity(_rc.get_desired_vertical_velocity());
    _logger.update_rc_inputs(_rc.get_throttle_in_pwm(),
                             _rc.get_roll_in_pwm(),
                             _rc.get_pitch_in_pwm(),
                             _rc.get_yaw_in_pwm());
    _logger.update_motor_emergency(_rc.is_motor_emergency());
}

void Copter::read_inertial_sensor()
{
    _inertialSensor.read();

    _attitudeController.set_measured_rates(_inertialSensor.getCalibGyroX(),
                                           _inertialSensor.getCalibGyroY(),
                                           _inertialSensor.getCalibGyroZ());
    _logger.update_gyro(_inertialSensor.getCalibGyroX(),
                        _inertialSensor.getCalibGyroY(),
                        _inertialSensor.getCalibGyroZ());
    _logger.update_accelerometer(_inertialSensor.getCalibAccelX(),
                                 _inertialSensor.getCalibAccelY(),
                                 _inertialSensor.getCalibAccelZ());
}

void Copter::read_barometer()
{
    _barometer.read();
}

void Copter::check_takeoff()
{
    if (!is_flying && _motors.is_armed() && (_verticalEstimator.get_estimated_vertical_velocity() > 30.0))
    {
        is_flying = true;
        _logger.update_flying(is_flying);
    }
    if (_rc.get_throttle_in_pwm() < 1050)
    {
        is_flying = false;
        _logger.update_flying(is_flying);
    }
}

void Copter::check_pid_reset()
{
    if (_rc.get_throttle_in_pwm() < 1050)
    {
        _attitudeController.reset();
        _verticalVelocityController.reset();
    }
}

void Copter::run_main_controller()
{
    _attitudeEstimator.update();
    _verticalEstimator.update();

    _attitudeController.update_angle_control(_attitudeEstimator.get_estimated_roll(), _attitudeEstimator.get_estimated_pitch());
    //_attitudeController.update_rate_control();
    _verticalVelocityController.update(_verticalEstimator.get_estimated_vertical_velocity());

    float roll_command = _attitudeController.get_roll_command();
    float pitch_command = _attitudeController.get_pitch_command();
    float yaw_command = _attitudeController.get_yaw_command();
    float hover_command = _verticalVelocityController.get_hover_command();
    float throttle_command = _rc.get_mid_throttle() + hover_command;
    // float throttle_command = _rc.get_throttle_in_pwm();

    _motors.set_command_inputs(throttle_command, roll_command, pitch_command, yaw_command);

    _logger.update_estimated_angles(_attitudeEstimator.get_estimated_roll(), _attitudeEstimator.get_estimated_pitch());
    _logger.update_vertical(_verticalEstimator.get_estimated_vertical_velocity(),
                            _verticalEstimator.get_estimated_altitude_in_cm());
    _logger.update_commands(throttle_command, roll_command, pitch_command, yaw_command, hover_command);
}

void Copter::run_motors()
{
    _motors.update_outputs();
    _motors.write_to_motors();
    _motors.write_logs();
}

void Copter::run_battery_monitor()
{
    _battMonitor.monitor();
    _logger.update_voltage_current(_battMonitor.voltage(), _battMonitor.current());
    _logger.update_batt_failsafe(_battMonitor.is_batt_failsafe());
}

void Copter::check_esc_calibration()
{

    // i = 4 was not enough, i = 25 was not enough, i = 50 is not enough, still not reading escape
    uint8_t i = 0;
    while ((i++ < 100) && !_rc.has_received_radio_reading())
    {
        _rc.read();
        delay(20); // From test, the 20ms delay is enough for capturing the radio reading
    }

    if (_storage.check_for_esc_calibration())
    {

        if (_rc.get_throttle_in_pwm() >= ESC_CALIBRATION_HIGH_THROTTLE)
        {
            _storage.set_check_esc_calibration(false);
            StatusNotifier::events.esc_calibration = true;

            // Keep in the loop until drone is reboot
            while (1)
            {
                update_notifier();
                delay(4);
            }
        }
    }
    else
    {
        if (_rc.get_throttle_in_pwm() >= ESC_CALIBRATION_HIGH_THROTTLE)
        {
            _storage.set_check_esc_calibration(true);
            _motors.setArm(true);
            while (1)
            {
                update_notifier();
                _rc.read();
                float throttleInput = _rc.get_throttle_in_pwm();
                _motors.set_esc_calibration_throttle(throttleInput);
                _motors.write_to_motors();
                delay(4);
            }
        }
    }
}

void Copter::check_motors_startup()
{
    arm_esc_at_minimum();

    _motors.setArm(true);

    uint32_t start_ms = millis();
    uint32_t test_interval_ms = 10000;
    while (millis() - start_ms < test_interval_ms)
    {
        _motors.set_esc_calibration_throttle(MOTORS_MINIMUM_STARTUP_THROTTLE);
        _motors.write_to_motors();
        update_notifier();
        delay(4);
    }
    _motors.setArm(false);

    arm_esc_at_minimum();
}

void Copter::check_motors_mapping()
{
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
                _motors.set_motor_to_stop();
            }
            _motors.write_to_motors();
            delay(4);
            now_ms = millis();
        }
    }

    _motors.setArm(false);

    arm_esc_at_minimum();
}

void Copter::arm_esc_at_minimum()
{
    _motors.setArm(true);

    uint32_t start_ms = millis();
    uint32_t test_interval_ms = 3000;
    while (millis() - start_ms < test_interval_ms)
    {
        _motors.run_motors_at_minimum();
        update_notifier();
        delay(4);
    }

    _motors.setArm(false);
}

void Copter::check_motors_arming()
{
    if (_rc.get_throttle_in_pwm() > 1005)
    {
        _arming_counter = 0;
        return;
    }

    uint16_t yaw_in_pwm = _rc.get_yaw_in_pwm();
    if (yaw_in_pwm >= 1992)
    {
        if (_arming_counter < ARM_DELAY)
        {
            _arming_counter++;
        }

        if (_arming_counter == ARM_DELAY && !_motors.is_armed())
        {
            if (is_pre_arm_check_pass())
            {
                _motors.setArm(true);
                _motors.run_motors_at_minimum();
                _logger.update_arming(true);
            }
            else
            {
                _arming_counter = 0;
            }
        }
    }
    else if (yaw_in_pwm <= 1008)
    {
        if (_arming_counter <= DISARM_DELAY)
        {
            _arming_counter++;
        }

        if (_arming_counter == DISARM_DELAY && _motors.is_armed())
        {
            _motors.run_motors_at_minimum();
            _motors.setArm(false);
            _logger.update_arming(false);
        }
    }
    else
    {
        _arming_counter = 0;
    }
}

bool Copter::is_pre_arm_check_pass()
{
    return !_battMonitor.is_batt_failsafe();
}

void Copter::run_notifier()
{
    _notifier.update();
}

void Copter::update_notifier()
{
    uint32_t now = millis();
    if (now - notifier_update_ms > 20)
    {
        notifier_update_ms = now;
        _notifier.update();
    }
}

void Copter::insert_log_entry_to_buffer()
{
    _logger.insert_log_entry_to_buffer();
}

void Copter::write_logs_to_sd()
{
    _logger.write_logs_to_sd();
}

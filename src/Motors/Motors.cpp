#include "Motors.h"
#include "Axis.h"
#include <Wire.h>
#include "../Notify/StatusNotifier.h"
#include "../Logger/Logger.h"

#define MAX_THROTTLE 1999
#define IDLE_THROTTLE 1180
#define CUT_OFF_THROTTLE 1000
#define SAFE_MAX_THROTTLE 1800
#define SAFE_MIN_THROTTLE 1050

void Motors::init()
{
    _escOutput.init();
}

void Motors::update_outputs()
{
    compute_mixer_outputs();
    apply_output_logic();
    compute_final_outputs();
    update_esc_outputs();
}

void Motors::set_command_inputs(float throttle_command, float roll_command, float pitch_command, float yaw_command)
{
    _command_inputs[Input::THROTTLE] = throttle_command;
    _command_inputs[Input::ROLL] = roll_command;
    _command_inputs[Input::PITCH] = pitch_command;
    _command_inputs[Input::YAW] = yaw_command;

    if (_command_inputs[Input::THROTTLE] > SAFE_MAX_THROTTLE)
    {
        _command_inputs[Input::THROTTLE] = SAFE_MAX_THROTTLE;
    }
}

void Motors::compute_mixer_outputs()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < NUM_MOTORS; j++)
        {
            sum += _mixer[i][j] * _command_inputs[j];
        }
        _mixed_motor_outputs[i] = sum;
    }
}

void Motors::apply_output_logic()
{
    if (!is_armed())
    {
        _spoolState = SpoolState::SHUT_DOWN;
    }
    else if (_is_motor_emergency)
    {
        _spoolState = SpoolState::SHUT_DOWN;
    }
    else if (_throttle_radio < SAFE_MIN_THROTTLE)
    {
        _spoolState = SpoolState::SHUT_DOWN;
    }
    else if (_throttle_radio < IDLE_THROTTLE)
    {
        _spoolState = SpoolState::GROUND_IDLE;
    }
    else
    {
        _spoolState = SpoolState::THROTTLE_UNLIMITED;
    }
}

void Motors::compute_final_outputs()
{
    switch (_spoolState)
    {
    case SpoolState::SHUT_DOWN:
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _motor_outputs[i] = CUT_OFF_THROTTLE;
        }
        break;

    case SpoolState::GROUND_IDLE:
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _motor_outputs[i] = IDLE_THROTTLE;
        }
        break;

    case SpoolState::THROTTLE_UNLIMITED:
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _motor_outputs[i] = _mixed_motor_outputs[i];
            _motor_outputs[i] = constrain(_motor_outputs[i], 1000.0f, 1999.0f); // Constrain to valid PWM range

            if (_motor_outputs[i] < IDLE_THROTTLE)
                _motor_outputs[i] = IDLE_THROTTLE;
        }
        break;
    }
}

void Motors::update_esc_outputs()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        _escOutput.set_pwm_value(i, _motor_outputs[i]);
    }
}

void Motors::setArm(bool arm)
{
    if (arm != _armed)
    {
        _armed = arm;
        StatusNotifier::events.armed = arm;
    }
}

void Motors::set_motor_emergency(bool motor_emergency)
{
    _is_motor_emergency = motor_emergency;
}

void Motors::set_esc_calibration_throttle(float throttle)
{
    if (is_armed())
    {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _escOutput.set_pwm_value(i, throttle);
        }
    }
}

void Motors::set_motor_sequence_throttle(int sequence, float throttle)
{
    if (is_armed())
    {
        _escOutput.set_pwm_value(sequence - 1, throttle);
    }
}

void Motors::set_motor_to_stop()
{
    if (is_armed())
    {
        float cut_off_throttle = CUT_OFF_THROTTLE;
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _escOutput.set_pwm_value(i, cut_off_throttle);
        }
    }
}

void Motors::write_to_motors()
{
    _escOutput.write_pwm_outputs();
}

void Motors::write_logs()
{
    Logger::get_singleton().update_motors(
        _motor_outputs[0],
        _motor_outputs[1],
        _motor_outputs[2],
        _motor_outputs[3]);
}

void Motors::set_throttle_radio(float throttle_input)
{
    _throttle_radio = throttle_input;
}

void Motors::run_motors_at_minimum()
{
    set_motor_to_stop();
    write_to_motors();
}

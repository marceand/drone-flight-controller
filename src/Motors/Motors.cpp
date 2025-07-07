#include "Motors.h"
#include "MotorsMixer.h"
#include "Axis.h"
#include <Wire.h>

#define MAX_THROTTLE 1999
#define IDLE_THROTTLE 1180
#define CUT_OFF_THROTTLE 1000
#define SAFE_MAX_THROTTLE 1800
#define SAFE_MIN_THROTTLE 1050

void Motors::init()
{
    _escOutput.init();
}

void Motors::runMotors(float throttleInput, float rollInput, float pitchInput, float yawInput)
{

    // Serial.print("Roll:");
    // Serial.print(rollInput);
    // Serial.print("\t");
    // Serial.print("Pitch:");
    // Serial.print(pitchInput);
    // Serial.print("\t");
    // Serial.print("Throttle:");
    // Serial.print(throttleInput);
    // Serial.print("\t");
    // Serial.print("Yaw:");
    // Serial.println(yawInput);

    float motor_1_output = calculateMotorOutput(MotorsMixer::motor_1_mixer, throttleInput, rollInput, pitchInput, yawInput);
    float motor_2_output = calculateMotorOutput(MotorsMixer::motor_2_mixer, throttleInput, rollInput, pitchInput, yawInput);
    float motor_3_output = calculateMotorOutput(MotorsMixer::motor_3_mixer, throttleInput, rollInput, pitchInput, yawInput);
    float motor_4_output = calculateMotorOutput(MotorsMixer::motor_4_mixer, throttleInput, rollInput, pitchInput, yawInput);

    // Serial.print("M1:");
    // Serial.print(motor_1_output);
    // Serial.print("\t");
    // Serial.print("M2:");
    // Serial.print(motor_2_output);
    // Serial.print("\t");
    // Serial.print("M3:");
    // Serial.print(motor_3_output);
    // Serial.print("\t");
    // Serial.print("M4:");
    // Serial.println(motor_4_output);
    // float motor_1_output;
    // float motor_2_output;
    // float motor_3_output;
    // float motor_4_output;

    // if (throttleInput < SAFE_MIN_THROTTLE)
    // {

    //     motor_1_output = CUT_OFF_THROTTLE;
    //     motor_2_output = CUT_OFF_THROTTLE;
    //     motor_3_output = CUT_OFF_THROTTLE;
    //     motor_4_output = CUT_OFF_THROTTLE;
    // }
    // else
    // {
    //     if (throttleInput > SAFE_MAX_THROTTLE)
    //     {
    //         throttleInput = SAFE_MAX_THROTTLE;
    //     }

    //     motor_1_output = calculateMotorOutput(MotorsMixer::motor_1_mixer, throttleInput, rollInput, pitchInput, yawInput);
    //     motor_2_output = calculateMotorOutput(MotorsMixer::motor_2_mixer, throttleInput, rollInput, pitchInput, yawInput);
    //     motor_3_output = calculateMotorOutput(MotorsMixer::motor_3_mixer, throttleInput, rollInput, pitchInput, yawInput);
    //     motor_4_output = calculateMotorOutput(MotorsMixer::motor_4_mixer, throttleInput, rollInput, pitchInput, yawInput);
    // }

    // updateMotorOutputs(motor_1_output, motor_2_output, motor_3_output, motor_4_output);
}

void Motors::output_logic()
{
    if (!isArmed())
    {
        _spoolState = SpoolState::SHUT_DOWN;
    }
    else if (receiverThrottle < throttleGroundIdleThreshold)
    {
        _spoolState = SpoolState::GROUND_IDLE;
    }
    else
    {
        _spoolState = SpoolState::THROTTLE_UNLIMITED;
    }
}

void Motors::output_to_motors()
{
    switch (_spoolState)
    {
    case SpoolState::SHUT_DOWN:
        // Motors off (e.g., minimum PWM)
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _motor_outputs[i] = _escOutput.scale_ouput(CUT_OFF_THROTTLE);
        }
        break;

    case SpoolState::GROUND_IDLE:
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            _motor_outputs[i] = _escOutput.scale_ouput(IDLE_THROTTLE);
        }
        break;

    case SpoolState::THROTTLE_UNLIMITED:
        float minThrottle = _escOutput.scale_ouput(IDLE_THROTTLE);
        for (int i = 0; i < NUM_MOTORS; i++)
        {

            _motor_outputs[i] = _escOutput.scale_ouput(_mixed_motor_outputs[i]);
            _motor_outputs[i] = constrain(_motor_outputs[i], 1000.0f, 1999.0f); // Constrain to valid PWM range

            if (_motor_outputs[i] < IDLE_THROTTLE)
                _motor_outputs[i] = minThrottle;
        }
        break;
    }

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        _escOutput.write(i, _motor_outputs[i]);
    }
}

void Motors::calculate_ouput(float throttle_input, float roll_input, float pitch_input, float yaw_input)
{

    _command_inputs[Input::THROTTLE] = throttle_input;
    _command_inputs[Input::ROLL] = roll_input;
    _command_inputs[Input::PITCH] = pitch_input;
    _command_inputs[Input::YAW] = yaw_input;

    if (_command_inputs[Input::THROTTLE] > SAFE_MAX_THROTTLE)
    {
        _command_inputs[Input::THROTTLE] = SAFE_MAX_THROTTLE;
    }

    for (int i = 0; i < 4; i++)
    {
        float sum = 0;
        for (int j = 0; j < 4; j++)
        {
            sum += _mixer[i][j] * _command_inputs[j];
        }
        _mixed_motor_outputs[i] = sum;
    }
}

void Motors::setArm(bool arm)
{
    if (arm != _armed)
    {
        _armed = arm;
    }
}

void Motors::updateMotorOutputs(float motor_1_output, float motor_2_output, float motor_3_output, float motor_4_output)
{
    if (isArmed())
    {
        _escOutput.update_motor_1_speed(motor_1_output);
        _escOutput.update_motor_2_speed(motor_2_output);
        _escOutput.update_motor_3_speed(motor_3_output);
        _escOutput.update_motor_4_speed(motor_4_output);
    }
}

void Motors::runMotorsForESCPassthrough(float throttleInput)
{
    float throttle_input_scaled = applyResolutionScaleToOuput(throttleInput);
    updateMotorOutputs(throttle_input_scaled, throttle_input_scaled, throttle_input_scaled, throttle_input_scaled);
}

void Motors::runMotorInSequence(int motorSequence, float throttleInput)
{
    float throttle_input_scaled = applyResolutionScaleToOuput(throttleInput);
    if (isArmed())
    {
        switch (motorSequence)
        {
        case 1:
            _escOutput.update_motor_1_speed(throttle_input_scaled);
            break;
        case 2:
            _escOutput.update_motor_2_speed(throttle_input_scaled);
            break;
        case 3:
            _escOutput.update_motor_3_speed(throttle_input_scaled);
            break;
        case 4:
            _escOutput.update_motor_4_speed(throttle_input_scaled);
            break;
        default:
            break;
        }
    }
}

void Motors::runAtMinimum()
{
    float throttle_input_scaled = applyResolutionScaleToOuput(CUT_OFF_THROTTLE);
    updateMotorOutputs(throttle_input_scaled, throttle_input_scaled, throttle_input_scaled, throttle_input_scaled);
}

float Motors::calculateMotorOutput(MotorMixFunc mixer, float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    float output = mixer(throttleInput, rollInput, pitchInput, yawInput);
    output = applyResolutionScaleToOuput(output);
    return applyLimitToOutput(output);
}

float Motors::applyResolutionScaleToOuput(float throttle)
{
    return SCALE_TO_12_BIT * throttle;
}

float Motors::applyLimitToOutput(float throttle)
{
    if (throttle < IDLE_THROTTLE)
    {
        return IDLE_THROTTLE;
    }

    if (throttle > MAX_THROTTLE)
    {
        return MAX_THROTTLE;
    }

    return throttle;
}

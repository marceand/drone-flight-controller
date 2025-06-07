#include "Motors.h"
#include "MotorsMixer.h"

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
    float motor_1_output;
    float motor_2_output;
    float motor_3_output;
    float motor_4_output;

    if (throttleInput < SAFE_MIN_THROTTLE)
    {

        motor_1_output = CUT_OFF_THROTTLE;
        motor_2_output = CUT_OFF_THROTTLE;
        motor_3_output = CUT_OFF_THROTTLE;
        motor_4_output = CUT_OFF_THROTTLE;
    }
    else
    {
        if (throttleInput > SAFE_MAX_THROTTLE)
        {
            throttleInput = SAFE_MAX_THROTTLE;
        }

        motor_1_output = calculateMotorOutput(MotorsMixer::motor_1_mixer, throttleInput, rollInput, pitchInput, yawInput);
        motor_2_output = calculateMotorOutput(MotorsMixer::motor_2_mixer, throttleInput, rollInput, pitchInput, yawInput);
        motor_3_output = calculateMotorOutput(MotorsMixer::motor_3_mixer, throttleInput, rollInput, pitchInput, yawInput);
        motor_4_output = calculateMotorOutput(MotorsMixer::motor_4_mixer, throttleInput, rollInput, pitchInput, yawInput);
    }

    updateMotorOutputs(motor_1_output, motor_2_output, motor_3_output, motor_4_output);
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

#include "MotorsController.h"

void MotorsController::runMotors(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    if (throttleInput < SAFE_MIN_THROTTLE)
    {
        _escOutput.update_motor_1_speed(CUT_OFF_THROTTLE);
        _escOutput.update_motor_2_speed(CUT_OFF_THROTTLE);
        _escOutput.update_motor_3_speed(CUT_OFF_THROTTLE);
        _escOutput.update_motor_4_speed(CUT_OFF_THROTTLE);
        return;
    }

    if (throttleInput > SAFE_MAX_THROTTLE)
    {
        throttleInput = SAFE_MAX_THROTTLE;
    }

    float motor_1_output = motor_1_mixer(throttleInput, rollInput, pitchInput, yawInput);
    float motor_2_output = motor_2_mixer(throttleInput, rollInput, pitchInput, yawInput);
    float motor_3_output = motor_3_mixer(throttleInput, rollInput, pitchInput, yawInput);
    float motor_4_output = motor_4_mixer(throttleInput, rollInput, pitchInput, yawInput);

    motor_1_output = applyResolutionScale(motor_1_output);
    motor_2_output = applyResolutionScale(motor_2_output);
    motor_3_output = applyResolutionScale(motor_3_output);
    motor_4_output = applyResolutionScale(motor_4_output);

    motor_1_output = applyThrottleLimits(motor_1_output);
    motor_2_output = applyThrottleLimits(motor_2_output);
    motor_3_output = applyThrottleLimits(motor_3_output);
    motor_4_output = applyThrottleLimits(motor_4_output);

    _escOutput.update_motor_1_speed(motor_1_output);
    _escOutput.update_motor_2_speed(motor_2_output);
    _escOutput.update_motor_3_speed(motor_3_output);
    _escOutput.update_motor_4_speed(motor_4_output);
}

float MotorsController::motor_1_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput - rollInput - pitchInput - yawInput;
}

float MotorsController::motor_2_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput - rollInput + pitchInput + yawInput;
}

float MotorsController::motor_3_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput + rollInput + pitchInput - yawInput;
}

float MotorsController::motor_4_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput + rollInput - pitchInput - yawInput;
}

float MotorsController::applyResolutionScale(float throttle)
{
    return SCALE_TO_12_BIT * throttle;
}

float MotorsController::applyThrottleLimits(float throttle)
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

#include "MotorsMixer.h"

float MotorsMixer::motor_1_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput - rollInput - pitchInput - yawInput;
}

float MotorsMixer::motor_2_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput - rollInput + pitchInput + yawInput;
}

float MotorsMixer::motor_3_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput + rollInput + pitchInput - yawInput;
}

float MotorsMixer::motor_4_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput)
{
    return throttleInput + rollInput - pitchInput - yawInput;
}

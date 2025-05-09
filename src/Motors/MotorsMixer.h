#pragma once

class MotorsMixer
{

public:
    static float motor_1_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    static float motor_2_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    static float motor_3_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    static float motor_4_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
};
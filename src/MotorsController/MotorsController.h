#pragma once
#include "../HAL/ESCOutput.h"

#define MAX_THROTTLE 1999
#define IDLE_THROTTLE 1180
#define CUT_OFF_THROTTLE 1000
#define SAFE_MAX_THROTTLE 1800
#define SAFE_MIN_THROTTLE 1050

class MotorsController
{

public:
    void runMotors(float throttleInput, float rollInput, float pitchInput, float yawInput);

private:
    ESCOutput _escOutput;
    float motor_1_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    float motor_2_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    float motor_3_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    float motor_4_mixer(float throttleInput, float rollInput, float pitchInput, float yawInput);
    float applyResolutionScale(float throttle);
    float applyThrottleLimits(float throttle);
};
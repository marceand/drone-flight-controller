#pragma once

#include "../HAL/ESCOutput.h"

#define AP_MOTORS_NUM_MOTORS 4

class Motors
{
public:
    Motors(ESCOutput &ESCOutput) : _escOutput(ESCOutput) {};
    void init();
    bool isArmed()
    {
        return _armed;
    }
    void setArm(bool arm);
    void runMotors(float throttleInput, float rollInput, float pitchInput, float yawInput);
    void runMotorsForESCPassthrough(float throttleInput);
    void runMotorInSequence(int motorSequence, float throttleInput);
    void runAtMinimum();

private:
    float mixer[4][4] = {
        {1, -1, -1, -1}, // Motor 1
        {1, -1, 1, 1},   // Motor 2
        {1, 1, 1, -1},   // Motor 3
        {1, 1, -1, 1},   // Motor 4
    };
    typedef float (*MotorMixFunc)(float, float, float, float);
    ESCOutput &_escOutput;
    bool _armed;
    void updateMotorOutputs(float motor_1_output, float motor_2_output, float motor_3_output, float motor_4_output);
    float calculateMotorOutput(MotorMixFunc mixer, float throttleInput, float rollInput, float pitchInput, float yawInput);
    float applyResolutionScaleToOuput(float throttle);
    float applyLimitToOutput(float throttle);
};

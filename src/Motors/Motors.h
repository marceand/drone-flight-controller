#pragma once

#include "../HAL/ESCOutput.h"

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
    typedef float (*MotorMixFunc)(float, float, float, float);
    ESCOutput &_escOutput;
    bool _armed;
    void updateMotorOutputs(float motor_1_output, float motor_2_output, float motor_3_output, float motor_4_output);
    float calculateMotorOutput(MotorMixFunc mixer, float throttleInput, float rollInput, float pitchInput, float yawInput);
    float applyResolutionScaleToOuput(float throttle);
    float applyLimitToOutput(float throttle);
};

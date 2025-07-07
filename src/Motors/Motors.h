#pragma once

#include "../HAL/ESCOutput.h"

#define NUM_MOTORS 4

class Motors
{
public:
    Motors(ESCOutput &ESCOutput) : _escOutput(ESCOutput) {};
    enum class SpoolState
    {
        SHUT_DOWN = 0,
        GROUND_IDLE = 1,
        THROTTLE_UNLIMITED = 2,
    };
    void init();
    bool isArmed()
    {
        return _armed;
    }
    void setArm(bool arm);
    void runMotors(float throttleInput, float rollInput, float pitchInput, float yawInput);
    void calculate_ouput(float throttle_input, float roll_input, float pitch_input, float yaw_nput);
    void runMotorsForESCPassthrough(float throttleInput);
    void runMotorInSequence(int motorSequence, float throttleInput);
    void runAtMinimum();

private:
    SpoolState _spoolState = SpoolState::SHUT_DOWN;
    float _command_inputs[NUM_MOTORS] = {0.0f};
    float _mixed_motor_outputs[NUM_MOTORS] = {0.0f};
    float _motor_outputs[NUM_MOTORS] = {1000.0f};
    float _mixer[4][4] = {
        {1, -1, -1, -1}, // Motor 1
        {1, -1, 1, 1},   // Motor 2
        {1, 1, 1, -1},   // Motor 3
        {1, 1, -1, 1},   // Motor 4
    };
    typedef float (*MotorMixFunc)(float, float, float, float);
    ESCOutput &_escOutput;
    bool _armed;
    void output_logic();
    void output_to_motors();
    void updateMotorOutputs(float motor_1_output, float motor_2_output, float motor_3_output, float motor_4_output);
    float calculateMotorOutput(MotorMixFunc mixer, float throttleInput, float rollInput, float pitchInput, float yawInput);
    float applyResolutionScaleToOuput(float throttle);
    float applyLimitToOutput(float throttle);
};

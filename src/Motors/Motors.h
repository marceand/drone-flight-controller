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
    void update_outputs();
    void set_command_inputs(float throttle_command, float roll_command, float pitch_command, float yaw_command);
    void set_esc_calibration_throttle(float throttle);
    void set_motor_sequence_throttle(int sequence, float throttle);
    void set_motor_stop_throttle();
    void write_to_motors();
    void set_throttle_radio(float throttle_input);

private:
    ESCOutput &_escOutput;
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
    bool _armed;
    float _throttle_radio = 0.0f;
    void compute_mixer_outputs();
    void apply_output_logic();
    void compute_final_outputs();
    void update_esc_outputs();
};

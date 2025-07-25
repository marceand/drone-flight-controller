#pragma once

#include "../PID/CopterPID.h"

class AttitudeController
{
public:
    void reset();
    void set_integrator(bool enable);
    void set_desired_yaw_rate(float yaw_rate);
    void set_measured_rates(float roll_rate, float pitch_rate, float yaw_rate);
    void set_desired_angles(float roll_angle, float pitch_angle);
    void update(float roll_angle_estimated, float pitch_angle_estimated);
    float get_roll_command()
    {
        return _roll_command;
    }

    float get_pitch_command()
    {
        return _pitch_command;
    }

    float get_yaw_command()
    {
        return _yaw_command;
    }

private:
    CopterPID &_rateRollPID;
    CopterPID &_ratePitchPID;
    CopterPID &_rateYawPID;
    CopterPID &_angleRollPID;
    CopterPID &_anglePitchPID;
    float _roll_rate_desired = 0.0f;
    float _pitch_rate_desired = 0.0f;
    float _yaw_rate_desired = 0.0f;
    float _roll_rate_measured = 0.0f;
    float _pitch_rate_measured = 0.0f;
    float _yaw_rate_measured = 0.0f;
    float _roll_angle_desired = 0.0f;
    float _pitch_angle_desired = 0.0f;
    float _roll_command = 0.0f;
    float _pitch_command = 0.0f;
    float _yaw_command = 0.0f;
};

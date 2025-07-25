#pragma once
#include "../PID/CopterPID.h"

class VerticalVelocityController
{
public:
    void set_integrator(bool enable);
    void set_mid_throttle(float mid_throttle)
    {
        _mid_throttle = mid_throttle;
    }
    void set_desired_vertical_velocity(float vertical_velocity)
    {
        _vertical_velocity_desired = vertical_velocity;
    }
    void update(float vertical_velocity_estimated);
    float get_throttle_command()
    {
        return _throttle_command;
    }

private:
    CopterPID &_velocityController;
    float _mid_throttle = 0.0f;
    float _vertical_velocity_desired = 0.0f;
    float _throttle_command = 0.0f;
};

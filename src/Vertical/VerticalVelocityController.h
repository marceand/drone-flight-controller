#pragma once
#include "../PID/CopterPID.h"

class VerticalVelocityController
{
public:
    VerticalVelocityController(CopterPID &velocityPID) : _velocityPID(velocityPID)
    {
    }
    void set_parameters();
    void reset();
    void set_integrator(bool enable);
    void set_desired_vertical_velocity(float vertical_velocity)
    {
        _vertical_velocity_desired = vertical_velocity;
    }
    void update(float vertical_velocity_estimated);
    float get_hover_command()
    {
        return _hover_command;
    }

private:
    CopterPID &_velocityPID;
    float _vertical_velocity_desired = 0.0f;
    float _hover_command = 0.0f;
};

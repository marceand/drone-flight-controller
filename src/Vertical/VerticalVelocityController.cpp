#include "VerticalVelocityController.h"

void VerticalVelocityController::set_parameters()
{
    _velocityPID.setParameters(3.5, 0.0015, 0.01, 0.004, 400, 400);
}

void VerticalVelocityController::reset()
{
    _velocityPID.reset();
}

void VerticalVelocityController::set_integrator(bool enable)
{
    _velocityPID.set_integrator(enable);
}

void VerticalVelocityController::update(float vertical_velocity_estimated)
{
    _hover_command = _velocityPID.computePID(_vertical_velocity_desired, vertical_velocity_estimated);
}

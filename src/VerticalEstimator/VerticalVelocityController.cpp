#include "VerticalVelocityController.h"

void VerticalVelocityController::set_integrator(bool enable)
{
    _velocityController.set_integrator(enable);
}

void VerticalVelocityController::update(float vertical_velocity_estimated)
{
    float throttle_hover = _velocityController.computePID(_vertical_velocity_desired, vertical_velocity_estimated);
    _throttle_command = _mid_throttle + throttle_hover;
}

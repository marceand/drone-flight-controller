#include "VerticalVelocityController.h"
#include "../Logger/Logger.h"

#define VERTICAL_VELOCITY_KP 3.5f
#define VERTICAL_VELOCITY_KI 0.0015f
#define VERTICAL_VELOCITY_KD 0.01f

void VerticalVelocityController::set_parameters()
{
    _velocityPID.setParameters(VERTICAL_VELOCITY_KP, VERTICAL_VELOCITY_KI, VERTICAL_VELOCITY_KD, 0.004, 400, 400);
    Logger::get_singleton().update_vertical_velocity_pid_gains(VERTICAL_VELOCITY_KP, VERTICAL_VELOCITY_KI, VERTICAL_VELOCITY_KD);
}

void VerticalVelocityController::reset()
{
    _velocityPID.reset();
}

void VerticalVelocityController::update(float vertical_velocity_estimated)
{
    _hover_command = _velocityPID.computePID(_vertical_velocity_desired, vertical_velocity_estimated);
}

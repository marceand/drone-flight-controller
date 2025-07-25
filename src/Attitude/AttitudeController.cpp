#include "AttitudeController.h"

void AttitudeController::reset()
{
    _angleRollPID.reset();
    _anglePitchPID.reset();
    _rateRollPID.reset();
    _ratePitchPID.reset();
    _rateYawPID.reset();
}

void AttitudeController::set_integrator(bool enable)
{
    _angleRollPID.set_integrator(enable);
    _anglePitchPID.set_integrator(enable);
    _rateRollPID.set_integrator(enable);
    _ratePitchPID.set_integrator(enable);
    _rateYawPID.set_integrator(enable);
}

void AttitudeController::set_desired_yaw_rate(float yaw_rate)
{
    _yaw_rate_desired = yaw_rate;
}

void AttitudeController::set_measured_rates(float roll_rate, float pitch_rate, float yaw_rate)
{
    _roll_rate_measured = roll_rate;
    _pitch_rate_measured = pitch_rate;
    _yaw_rate_measured = yaw_rate;
}

void AttitudeController::set_desired_angles(float roll_angle, float pitch_angle)
{
    _roll_angle_desired = roll_angle;
    _pitch_angle_desired = pitch_angle;
}

void AttitudeController::update(float roll_angle_estimated, float pitch_angle_estimated)
{
    _roll_rate_desired = _angleRollPID.computePID(_roll_angle_desired, roll_angle_estimated);
    _pitch_rate_desired = _anglePitchPID.computePID(_pitch_angle_desired, pitch_angle_estimated);

    _roll_command = _rateRollPID.computePID(_roll_rate_desired, _roll_rate_measured);
    _pitch_command = _ratePitchPID.computePID(_pitch_rate_desired, _pitch_rate_measured);
    _yaw_command = _rateYawPID.computePID(_yaw_rate_desired, _yaw_rate_measured);
}
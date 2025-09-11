#include "VerticalEstimator.h"

void VerticalEstimator::set_parameters()
{
    _altitudeVelocityKF.setParameters();
}

void VerticalEstimator::update()
{
    calculateRawVerticalAcceleration();
    estimateVerticalVelocity();
}

void VerticalEstimator::calculateRawVerticalAcceleration()
{
    float accelX = _inertialSensor.getCalibAccelX();
    float accelY = _inertialSensor.getCalibAccelY();
    float accelZ = _inertialSensor.getCalibAccelZ();
    float roll_angle = _attitudeEstimator.get_raw_roll();
    float pitch_angle = _attitudeEstimator.get_raw_pitch();

    float accel_z_inertial = -sin(pitch_angle * (3.142 / 180)) * accelX +
                             cos(pitch_angle * (3.142 / 180)) * sin(roll_angle * (3.142 / 180)) * accelY +
                             cos(pitch_angle * (3.142 / 180)) * cos(roll_angle * (3.142 / 180)) * accelZ;

    _raw_vertical_acceleration = (accel_z_inertial - 1.0) * 9.81 * 100; // cm/s^2
}

void VerticalEstimator::estimateVerticalVelocity()
{
    float relative_altitude = _barometer.get_relative_altitude_in_cm();
    _estimated_vertical_velocity = _altitudeVelocityKF.calculateVerticalVelocity(relative_altitude, _raw_vertical_acceleration);
    _estimated_altitude = _altitudeVelocityKF.getAltitude();
}

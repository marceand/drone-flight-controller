#include "AttitudeEstimator.h"

void AttitudeEstimator::update()
{
    float accelX = _inertialSensor.getCalibAccelX();
    float accelY = _inertialSensor.getCalibAccelY();
    float accelZ = _inertialSensor.getCalibAccelZ();
    _raw_roll_angle = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * 1 / (3.142 / 180);
    _raw_pitch_angle = -atan(accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 1 / (3.142 / 180);

    float roll_rate = _inertialSensor.getCalibGyroX();
    float pitch_rate = _inertialSensor.getCalibGyroY();
    _estimated_roll_angle = _rollKF.calculateAngle(roll_rate, _raw_roll_angle);
    _estimated_pitch_angle = _pitchKF.calculateAngle(pitch_rate, _raw_pitch_angle);
}

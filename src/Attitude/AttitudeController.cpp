#include "AttitudeController.h"
#include "../Logger/Logger.h"

#define ROLL_ANGLE_KP 1.2f
// #define ROLL_ANGLE_KP 2.0f
#define ROLL_ANGLE_KI 0.0f
#define ROLL_ANGLE_KD 0.0f

#define PITCH_ANGLE_KP 1.2f
// #define PITCH_ANGLE_KP 2.0f
#define PITCH_ANGLE_KI 0.0f
#define PITCH_ANGLE_KD 0.0f

#define ROLL_RATE_KP 0.62f
#define ROLL_RATE_KI 1.8f
#define ROLL_RATE_KD 0.02f

#define PITCH_RATE_KP 0.62f
#define PITCH_RATE_KI 1.8f
#define PITCH_RATE_KD 0.02f

// #define YAW_RATE_KP 0.9f
// #define YAW_RATE_KI 0.7f
// #define YAW_RATE_KD 0.0f
// #define YAW_RATE_KP 1.25f
// #define YAW_RATE_KI 0.7f
// #define YAW_RATE_KD 0.0f
#define YAW_RATE_KP 0.9f
#define YAW_RATE_KI 1.25f
#define YAW_RATE_KD 0.0f

void AttitudeController::reset()
{
    _angleRollPID.reset();
    _anglePitchPID.reset();
    _rateRollPID.reset();
    _ratePitchPID.reset();
    _rateYawPID.reset();
}

void AttitudeController::set_parameters()
{

    _angleRollPID.setParameters(ROLL_ANGLE_KP, ROLL_ANGLE_KI, ROLL_ANGLE_KD, 0.004, 400, 400);
    _anglePitchPID.setParameters(PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD, 0.004, 400, 400);
    _rateRollPID.setParameters(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD, 0.004, 400, 400);
    _ratePitchPID.setParameters(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD, 0.004, 400, 400);
    _rateYawPID.setParameters(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, 0.004, 400, 400); // cancel yawing

    Logger::get_singleton().update_roll_angle_pid_gains(ROLL_ANGLE_KP, ROLL_ANGLE_KI, ROLL_ANGLE_KD);
    Logger::get_singleton().update_pitch_angle_pid_gains(PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD);
    Logger::get_singleton().update_roll_rate_pid_gains(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD);
    Logger::get_singleton().update_pitch_rate_pid_gains(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD);
    Logger::get_singleton().update_yaw_rate_pid_gains(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD);

    // Test 1: it vibrate or jerk because of saturation
    //  first test, the drone do sudden jerk or oscillate
    //  no yaw now, the yaw pid is working
    // _angleRollPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _rateRollPID.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    // _ratePitchPID.setParameters(0.6, 3.5, 0.03, 0.004, 400, 400);
    // _rateYawPID.setParameters(2.5, 11.3, 0, 0.004, 400, 400); // cancel yawing

    // this work
    // _angleRollPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _rateRollPID.setParameters(0.62, 1.8, 0.02, 0.004, 400, 400);
    // _ratePitchPID.setParameters(0.62, 1.8, 0.02, 0.004, 400, 400);
    // _rateYawPID.setParameters(0.9, 0.7, 0, 0.004, 400, 400); // cancel yawing

    // Test 2: it is not stable, it drift a lot
    // _angleRollPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _rateRollPID.setParameters(0.35, 2.1, 0.03, 0.004, 400, 400);
    // _ratePitchPID.setParameters(0.35, 2.1, 0.03, 0.004, 400, 400);
    // _rateYawPID.setParameters(2.5, 11.3, 0, 0.004, 400, 400); // cancel yawing

    // Test 3:
    // _angleRollPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _rateRollPID.setParameters(0.62, 2.1, 0.008, 0.004, 400, 400);
    // _ratePitchPID.setParameters(0.62, 2.1, 0.008, 0.004, 400, 400);
    // _rateYawPID.setParameters(2.5, 11.3, 0, 0.004, 400, 400); // cancel yawing

    //_rateYawPID.setParameters(4, 3, 0, 0.004, 400, 400); // normal yawing
    //_rateYawPID.setParameters(3, 2, 0, 0.004, 400, 400); // still yawing
    //_rateYawPID.setParameters(4, 3.5, 0, 0.004, 400, 400); // little yawing
    //_rateYawPID.setParameters(2.5, 11.3, 0, 0.004, 400, 400); // cancel yawing

    // I tried this set
    // _angleRollPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchPID.setParameters(2.0, 0.0, 0.0, 0.004, 400, 400);
    // _rateRollPID.setParameters(0.5, 3.5, 0.03, 0.004, 400, 400);
    // _ratePitchPID.setParameters(0.5, 3.5, 0.03, 0.004, 400, 400);

    //_rateYawPID.setParameters(0.5, 0.05, 0, 0.004, 400, 400); // already tried
    //_rateYawPID.setParameters(1.2, 11.3, 0, 0.004, 400, 400);  // already tried
    //_rateYawPID.setParameters(0.180f, 0.018f, 0, 0.004, 400, 400); // already tried

    //_rateYawPID.setParameters(0.180f, 0.018f, 0, 0.004, 400, 400); // already tried
    //_rateYawPID.setParameters(0.198f, 0.018f, 0, 0.004, 400, 400); // already tried
    //_rateYawPID.setParameters(0.218f, 0.018f, 0, 0.004, 400, 400); // already tried
    // _rateYawPID.setParameters(0.240f, 0.018f, 0, 0.004, 400, 400); // already tried
    // _rateYawPID.setParameters(0.265f, 0.018f, 0, 0.004, 400, 400); // already tried
    // _rateYawPID.setParameters(0.425f, 0.018f, 0, 0.004, 400, 400); // already tried
    //_rateYawPID.setParameters(0.513f, 0.018f, 0, 0.004, 400, 400); // already tried
    // _rateYawPID.setParameters(0.825f, 0.018f, 0, 0.004, 400, 400); // already tried
    // _rateYawPID.setParameters(0.997f, 0.018f, 0, 0.004, 400, 400); // already tried
    //_rateYawPID.setParameters(1.325f, 0.018f, 0, 0.004, 400, 400); // already tried
    //_rateYawPID.setParameters(1.762f, 0.018f, 0, 0.004, 400, 400); // already tried

    // second test
    // _angleRollPID.setParameters(1.6, 0.0, 0.0, 0.004, 400, 400);
    // _anglePitchPID.setParameters(1.6, 0.0, 0.0, 0.004, 400, 400);
    // _rateRollPID.setParameters(0.4, 3.2, 0.03, 0.004, 400, 400);
    // _ratePitchPID.setParameters(0.4, 3.2, 0.03, 0.004, 400, 400);
    // _rateYawPID.setParameters(0.180f, 0.018f, 0, 0.004, 400, 400);
}

void AttitudeController::set_desired_rates(float roll_rate, float pitch_rate, float yaw_rate)
{
    _roll_rate_desired = roll_rate;
    _pitch_rate_desired = pitch_rate;
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

void AttitudeController::update_rate_control()
{
    _roll_command = _rateRollPID.computePID(_roll_rate_desired, _roll_rate_measured);
    _pitch_command = _ratePitchPID.computePID(_pitch_rate_desired, _pitch_rate_measured);
    _yaw_command = _rateYawPID.computePID(_yaw_rate_desired, _yaw_rate_measured);
}

void AttitudeController::update_angle_control(float roll_angle_estimated, float pitch_angle_estimated)
{
    float roll_rate_desired_from_pid = _angleRollPID.computePID(_roll_angle_desired, roll_angle_estimated);
    float pitch_rate_desired_from_pid = _anglePitchPID.computePID(_pitch_angle_desired, pitch_angle_estimated);

    _roll_command = _rateRollPID.computePID(roll_rate_desired_from_pid, _roll_rate_measured);
    _pitch_command = _ratePitchPID.computePID(pitch_rate_desired_from_pid, _pitch_rate_measured);
    _yaw_command = _rateYawPID.computePID(_yaw_rate_desired, _yaw_rate_measured);
}
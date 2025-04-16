
#pragma once

#include <sbus.h>

#define RC_CHANNEL_IDX_ROLL 0
#define RC_CHANNEL_IDX_PITCH 1
#define RC_CHANNEL_IDX_THROTTLE 2
#define RC_CHANNEL_IDX_YAW 3

// The range of a channel's possible values (microseconds)
#define RC_CHANNEL_DEFAULT_VAL 1000
#define RC_MIN_CHANNEL_VALUE 1000
#define RC_MAX_CHANNEL_VALUE 2000
#define RC_MID_CHANNEL_VALUE 1500
#define RC_MAX_THROTTLE 1800
#define RC_MIN_THROTTLE 1050

#define MIN_PWM 1000
#define MAX_PWM 2000
#define MIN_SBUS 172
#define MAX_SBUS 1811

#define DESIRED_GYRO_FACTOR 0.15f
#define DESIRED_ANGLE_FACTOR 0.10f


class RC_Channels
{
private:
    struct pwm_channels_t
    {
        uint16_t roll;
        uint16_t pitch;
        uint16_t throttle;
        uint16_t yaw;
    };

    struct desired_rates_t
    {
        float roll;
        float pitch;
        float yaw;
    };

    struct desired_angles_t
    {
        float roll;
        float pitch;
    };

    pwm_channels_t _pwm_channels = {RC_CHANNEL_DEFAULT_VAL, RC_CHANNEL_DEFAULT_VAL, RC_CHANNEL_DEFAULT_VAL, RC_CHANNEL_DEFAULT_VAL};
    desired_rates_t _desired_rates = {0.0, 0.0, 0.0};
    desired_angles_t _desired_angles = {0.0, 0.0};
    bfs::SbusRx _sbus_rx;
    uint16_t mapSbusToPWM(uint16_t sbusValue);
    float computeDesiredRate(uint16_t inputInPWM);
    float computeDesiredAngle(uint16_t inputInPWM);

public:
    RC_Channels(HardwareSerial *serial) : _sbus_rx(serial) {}
    void init();
    void read();
    uint16_t getRollInPWM() { return _pwm_channels.roll; }
    uint16_t getPitchInPWM() { return _pwm_channels.pitch; }
    uint16_t getThrottleInPWM() { return _pwm_channels.throttle; }
    uint16_t getYawInPWM() { return _pwm_channels.yaw; }
    float getDesiredRollRate() { return _desired_rates.roll; }
    float getDesiredPitchRate() { return _desired_rates.pitch; }
    float getDesiredYawRate() { return _desired_rates.yaw; }
    float getDesiredRollAngle() { return _desired_angles.roll; }
    float getDesiredPitchAngle() { return _desired_angles.pitch; }
};

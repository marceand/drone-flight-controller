
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
#define DESIRED_VELOCITY_FACTOR 0.3f

class RC_Channels
{
public:
    RC_Channels(HardwareSerial *serial) : _sbus_rx(serial)
    {
    }
    void init();
    void read();
    uint16_t getRollInPWM() { return _pwm_channels.roll; }
    uint16_t getPitchInPWM() { return _pwm_channels.pitch; }
    uint16_t getThrottleInPWM() { return _pwm_channels.throttle; }
    uint16_t getYawInPWM() { return _pwm_channels.yaw; }
    float getDesiredRollRate() { return computeDesiredRate(_pwm_channels.roll); }
    float getDesiredPitchRate() { return computeDesiredRate(_pwm_channels.pitch); }
    float getDesiredYawRate() { return computeDesiredRate(_pwm_channels.yaw); }
    float getDesiredRollAngle() { return computeDesiredAngle(_pwm_channels.roll); }
    float getDesiredPitchAngle() { return computeDesiredAngle(_pwm_channels.pitch); }
    float getDesiredThrottleVelocity() { return computeDesiredVelocity(_pwm_channels.throttle); }

private:
    struct pwm_channels_t
    {
        uint16_t roll;
        uint16_t pitch;
        uint16_t throttle;
        uint16_t yaw;
    };

    pwm_channels_t _pwm_channels = {RC_CHANNEL_DEFAULT_VAL, RC_CHANNEL_DEFAULT_VAL, RC_CHANNEL_DEFAULT_VAL, RC_CHANNEL_DEFAULT_VAL};
    bfs::SbusRx _sbus_rx;
    uint16_t mapSbusToPWM(uint16_t sbusValue);
    float computeDesiredRate(uint16_t inputInPWM);
    float computeDesiredAngle(uint16_t inputInPWM);
    float computeDesiredVelocity(uint16_t inputInPWM);
};

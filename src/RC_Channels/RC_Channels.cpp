#include "RC_Channels.h"

uint16_t RC_Channels::mapSbusToPWM(uint16_t sbusValue)
{
    if (sbusValue < MIN_SBUS)
    {
        sbusValue = MIN_SBUS;
    }

    if (sbusValue > MAX_SBUS)
    {
        sbusValue = MAX_SBUS;
    }

    uint16_t pwmValue = MIN_PWM + (sbusValue - MIN_SBUS) * (MAX_PWM - MIN_PWM) / (MAX_SBUS - MIN_SBUS);

    return pwmValue;
}

float RC_Channels::computeDesiredRate(uint16_t inputInPWM)
{
    return DESIRED_GYRO_FACTOR * (inputInPWM - RC_MID_CHANNEL_VALUE);
}

float RC_Channels::computeDesiredAngle(uint16_t inputInPWM)
{
    return DESIRED_ANGLE_FACTOR * (inputInPWM - RC_MID_CHANNEL_VALUE);

}

void RC_Channels::init()
{
    _sbus_rx.Begin();
}

void RC_Channels::read()
{
    if (_sbus_rx.Read())
    {
        bfs::SbusData data = _sbus_rx.data();

        uint16_t rollValue = mapSbusToPWM(data.ch[RC_CHANNEL_IDX_ROLL]);
        uint16_t pitchValue = mapSbusToPWM(data.ch[RC_CHANNEL_IDX_PITCH]);
        uint16_t throttleValue = mapSbusToPWM(data.ch[RC_CHANNEL_IDX_THROTTLE]);
        uint16_t yawValue = mapSbusToPWM(data.ch[RC_CHANNEL_IDX_YAW]);

        _pwm_channels.roll = constrain(rollValue, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.pitch = constrain(pitchValue, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.yaw = constrain(yawValue, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);

        uint16_t constrainThrottle = constrain(throttleValue, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.throttle = constrainThrottle > RC_MAX_THROTTLE ? RC_MAX_THROTTLE : constrainThrottle;

        _desired_rates.roll = computeDesiredRate(_pwm_channels.roll);
        _desired_rates.pitch = computeDesiredRate(_pwm_channels.pitch);
        _desired_rates.yaw = computeDesiredRate(_pwm_channels.yaw);
    }
}

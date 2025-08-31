#include "RC_Channels.h"

uint16_t RC_Channels::map_sbus_to_pwm(uint16_t sbus_value)
{
    if (sbus_value < MIN_SBUS)
    {
        sbus_value = MIN_SBUS;
    }

    if (sbus_value > MAX_SBUS)
    {
        sbus_value = MAX_SBUS;
    }

    uint16_t pwmValue = MIN_PWM + (sbus_value - MIN_SBUS) * (MAX_PWM - MIN_PWM) / (MAX_SBUS - MIN_SBUS);

    return pwmValue;
}

float RC_Channels::compute_desired_rate(uint16_t input_in_pwm)
{
    return DESIRED_GYRO_FACTOR * (input_in_pwm - RC_MID_CHANNEL_VALUE);
}

float RC_Channels::compute_desired_angle(uint16_t input_in_pwm)
{
    return DESIRED_ANGLE_FACTOR * (input_in_pwm - RC_MID_CHANNEL_VALUE);
}

float RC_Channels::compute_desired_velocity(uint16_t input_in_pwm)
{
    return DESIRED_VELOCITY_FACTOR * (input_in_pwm - RC_MID_CHANNEL_VALUE);
}

bool RC_Channels::is_motor_emergency(uint16_t aux_1, uint16_t aux_2)
{
    return (aux_1 >= RC_AUX_CHANNEL_HIGH_VALUE) && (aux_2 >= RC_AUX_CHANNEL_HIGH_VALUE);
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

        uint16_t roll = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_ROLL]);
        uint16_t pitch = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_PITCH]);
        uint16_t throttle = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_THROTTLE]);
        uint16_t yaw = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_YAW]);
        uint16_t aux_1 = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_AUX_1]);
        uint16_t aux_2 = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_AUX_2]);
        uint16_t aux_3 = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_AUX_3]);
        uint16_t aux_4 = map_sbus_to_pwm(data.ch[RC_CHANNEL_IDX_AUX_4]);

        _pwm_channels.roll = constrain(roll, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.pitch = constrain(pitch, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.throttle = constrain(throttle, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.yaw = constrain(yaw, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.aux_1 = constrain(aux_1, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.aux_2 = constrain(aux_2, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.aux_3 = constrain(aux_3, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
        _pwm_channels.aux_4 = constrain(aux_4, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
    }
}

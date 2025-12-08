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

float RC_Channels::compute_input_expo(uint16_t input_in_pwm, float expo)
{
    float input_normalize = (input_in_pwm - get_mid_throttle()) / 500.0f;
    return (1 - expo) * input_normalize / (1 - expo * abs(input_normalize));
}

bool RC_Channels::check_motor_emergency(uint16_t pwm)
{
    return pwm >= (RC_MAX_CHANNEL_VALUE - RC_DEAD_ZONE);
}

void RC_Channels::init()
{
    _sbus_rx.Begin();
}

void RC_Channels::read()
{
    const uint32_t now_ms = millis();

    if (_sbus_rx.Read())
    {
        bfs::SbusData data = _sbus_rx.data();

        for (int index = 0; index < NUM_CHANNELS; index++)
        {
            uint16_t pwm = map_sbus_to_pwm(data.ch[index]);
            pwm = constrain(pwm, RC_MIN_CHANNEL_VALUE, RC_MAX_CHANNEL_VALUE);
            channels[index].pwm = pwm;
        }

        _last_radio_reading_ms = now_ms;
        _has_received_radio_reading = true;
    }

    if (_radio_failsafe)
    {
        return;
    }

    if (!_has_received_radio_reading)
    {
        return;
    }

    const uint32_t radio_reading_elapsed_ms = now_ms - _last_radio_reading_ms;
    if (radio_reading_elapsed_ms < 1000)
    {
        return;
    }

    _radio_failsafe = true;
}

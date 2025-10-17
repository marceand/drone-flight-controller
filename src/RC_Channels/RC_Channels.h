
#pragma once

#include <sbus.h>

#define RC_CHANNEL_IDX_ROLL 0
#define RC_CHANNEL_IDX_PITCH 1
#define RC_CHANNEL_IDX_THROTTLE 2
#define RC_CHANNEL_IDX_YAW 3
#define RC_CHANNEL_IDX_AUX_1 4
#define RC_CHANNEL_IDX_AUX_2 5
#define RC_CHANNEL_IDX_AUX_3 6
#define RC_CHANNEL_IDX_AUX_4 7

// The range of a channel's possible values (microseconds)
#define RC_CHANNEL_DEFAULT_VAL 1000
#define RC_MIN_CHANNEL_VALUE 1000
#define RC_MAX_CHANNEL_VALUE 2000
#define RC_MID_CHANNEL_VALUE 1500
#define RC_AUX_CHANNEL_HIGH_VALUE 1900

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
        last_radio_reading_ms = millis();
    }
    void init();
    void read();
    uint16_t get_roll_in_pwm() { return _pwm_channels.roll; }
    uint16_t get_pitch_in_pwm() { return _pwm_channels.pitch; }
    uint16_t get_throttle_in_pwm() { return _pwm_channels.throttle; }
    uint16_t get_yaw_in_pwm() { return _pwm_channels.yaw; }
    uint16_t get_aux_1_in_pwm() { return _pwm_channels.aux_1; }
    uint16_t get_aux_2_in_pwm() { return _pwm_channels.aux_2; }
    uint16_t get_aux_3_in_pwm() { return _pwm_channels.aux_3; }
    uint16_t get_aux_4_in_pwm() { return _pwm_channels.aux_4; }
    float get_mid_throttle() { return RC_MID_CHANNEL_VALUE; }
    float get_desired_roll_rate() { return compute_desired_rate(_pwm_channels.roll); }
    float get_desired_pitch_rate() { return compute_desired_rate(_pwm_channels.pitch); }
    float get_desired_yaw_rate() { return -1.0 * compute_desired_rate(_pwm_channels.yaw); }
    float get_desired_roll_angle() { return compute_desired_angle(_pwm_channels.roll); }
    float get_desired_pitch_angle() { return compute_desired_angle(_pwm_channels.pitch); }
    float get_desired_vertical_velocity() { return compute_desired_velocity(_pwm_channels.throttle); }
    bool is_motor_emergency() { return is_motor_emergency(_pwm_channels.aux_1, _pwm_channels.aux_3); }
    bool is_radio_failsafe() { return radio_failsafe; }

private:
    struct pwm_channels_t
    {
        uint16_t roll;
        uint16_t pitch;
        uint16_t throttle;
        uint16_t yaw;
        uint16_t aux_1;
        uint16_t aux_2;
        uint16_t aux_3;
        uint16_t aux_4;
    };

    pwm_channels_t _pwm_channels = {RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL,
                                    RC_CHANNEL_DEFAULT_VAL};
    bfs::SbusRx _sbus_rx;
    bool radio_failsafe = false;
    bool has_received_radio_reading = false;
    uint32_t last_radio_reading_ms;
    uint16_t map_sbus_to_pwm(uint16_t sbus_value);
    float compute_desired_rate(uint16_t input_in_pwm);
    float compute_desired_angle(uint16_t input_in_pwm);
    float compute_desired_velocity(uint16_t input_in_pwm);
    bool is_motor_emergency(uint16_t aux_1, uint16_t aux_2);
};

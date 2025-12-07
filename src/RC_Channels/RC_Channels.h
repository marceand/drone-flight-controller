
#pragma once

#include <sbus.h>

#define NUM_CHANNELS 8
#define RC_CHANNEL_1 0
#define RC_CHANNEL_2 1
#define RC_CHANNEL_3 2
#define RC_CHANNEL_4 3
#define RC_CHANNEL_5 4
#define RC_CHANNEL_6 5
#define RC_CHANNEL_7 6
#define RC_CHANNEL_8 7

// The range of a channel's possible values (microseconds)
#define RC_CHANNEL_DEFAULT_VAL 1000
#define RC_MIN_CHANNEL_VALUE 1000
#define RC_MAX_CHANNEL_VALUE 2000
#define RC_MID_CHANNEL_VALUE 1500
#define RC_DEAD_ZONE 5

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
        _last_radio_reading_ms = millis();
    }
    void init();
    void read();
    uint16_t get_roll_in_pwm() { return channels[RC_CHANNEL_1].pwm; }
    uint16_t get_pitch_in_pwm() { return channels[RC_CHANNEL_2].pwm; }
    uint16_t get_throttle_in_pwm() { return channels[RC_CHANNEL_3].pwm; }
    uint16_t get_yaw_in_pwm() { return channels[RC_CHANNEL_4].pwm; }
    float get_mid_throttle() { return RC_MID_CHANNEL_VALUE; }
    float get_desired_roll_rate() { return compute_expo_desired_rate(get_roll_in_pwm()); }
    float get_desired_pitch_rate() { return compute_expo_desired_rate(get_pitch_in_pwm()); }
    float get_desired_yaw_rate() { return -1.0 * compute_expo_desired_rate(get_yaw_in_pwm()); }
    float get_desired_roll_angle() { return compute_desired_angle(get_roll_in_pwm()); }
    float get_desired_pitch_angle() { return compute_desired_angle(get_pitch_in_pwm()); }
    float get_desired_vertical_velocity() { return compute_desired_velocity(get_throttle_in_pwm()); }
    bool is_motor_emergency() { return check_motor_emergency(channels[RC_CHANNEL_8].pwm); }
    bool is_radio_failsafe() { return _radio_failsafe; }
    bool has_received_radio_reading()
    {
        return _has_received_radio_reading;
    }

private:
    struct channel
    {
        uint16_t pwm;
    };

    channel channels[NUM_CHANNELS] = {{RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL},
                                      {RC_CHANNEL_DEFAULT_VAL}};
    bfs::SbusRx _sbus_rx;
    bool _radio_failsafe = false;
    bool _has_received_radio_reading = false;
    uint32_t _last_radio_reading_ms;
    uint16_t map_sbus_to_pwm(uint16_t sbus_value);
    float compute_desired_rate(uint16_t input_in_pwm);
    float compute_expo_desired_rate(uint16_t input_in_pwm);
    float compute_desired_angle(uint16_t input_in_pwm);
    float compute_desired_velocity(uint16_t input_in_pwm);
    bool check_motor_emergency(uint16_t pwm);
};

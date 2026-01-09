#pragma once

#include <Arduino.h>
#include <SdFat.h>

#define RING_BUF_CAPACITY 400 * 512

class Logger
{
public:
    static Logger &get_singleton(void)
    {
        static Logger logger_singleton;
        return logger_singleton;
    }

    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    struct LogGains
    {
        uint16_t sync; // 0xA55A
        uint8_t type = 0;
        float roll_angle_kp = 0.0f, roll_angle_ki = 0.0f, roll_angle_kd = 0.0f;
        float pitch_angle_kp = 0.0f, pitch_angle_ki = 0.0f, pitch_angle_kd = 0.0f;
        float roll_rate_kp = 0.0f, roll_rate_ki = 0.0f, roll_rate_kd = 0.0f;
        float pitch_rate_kp = 0.0f, pitch_rate_ki = 0.0f, pitch_rate_kd = 0.0f;
        float yaw_rate_kp = 0.0f, yaw_rate_ki = 0.0f, yaw_rate_kd = 0.0f;
        float vertical_velocity_kp = 0.0f, vertical_velocity_ki = 0.0f, vertical_velocity_kd = 0.0f;
    };

    struct LogEntry
    {
        uint16_t sync; // 0xA55A
        uint8_t type = 1;
        uint32_t time_us = 0;
        uint16_t rc_throttle = 0, rc_roll = 0, rc_pitch = 0, rc_yaw = 0;
        float desired_roll_angle = 0.0f, desired_pitch_angle = 0.0f;
        float desired_roll_rate = 0.0f, desired_pitch_rate = 0.0f, desired_yaw_rate = 0.0f;
        float desired_vertical_velocity = 0.0f;
        float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
        float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
        float estimated_roll_angle = 0.0f, estimated_pitch_angle = 0.0f;
        float vertical_velocity = 0.0f;
        float altitude = 0.0f;
        float voltage = 0.0f, current = 0.0f;
        float cmd_throttle = 0.0f, cmd_roll = 0.0f, cmd_pitch = 0.0f, cmd_yaw = 0.0f, cmd_hover = 0.0f;
        float m1 = 0.0f, m2 = 0.0f, m3 = 0.0f, m4 = 0.0f;
        uint8_t is_flying = 0, is_armed = 0, is_radio_failsafe = 0, is_motor_emergency = 0, is_batt_failsafe = 0;
    };

    void init();
    void update_logging();
    void flush_log_to_sd();
    void update_rc_inputs(uint16_t rc_throttle, uint16_t rc_roll, uint16_t rc_pitch, uint16_t rc_yaw);
    void update_desired_angles(float desired_roll_angle, float desired_pitch_angle);
    void update_desired_rates(float desired_roll_rate, float desired_pitch_rate, float desired_yaw_rate);
    void update_desired_vertical_velocity(float desired_vertical_velocity);
    void update_gyro(float gx, float gy, float gz);
    void update_accelerometer(float ax, float ay, float az);
    void update_estimated_angles(float estimated_roll, float estimated_pitch);
    void update_vertical(float vz, float alt);
    void update_voltage_current(float voltage, float current);
    void update_commands(float cmd_throttle, float cmd_roll, float cmd_pitch, float cmd_yaw, float cmd_hover);
    void update_motors(float m1, float m2, float m3, float m4);
    void update_roll_angle_pid_gains(float kp, float ki, float kd);
    void update_pitch_angle_pid_gains(float kp, float ki, float kd);
    void update_roll_rate_pid_gains(float kp, float ki, float kd);
    void update_pitch_rate_pid_gains(float kp, float ki, float kd);
    void update_yaw_rate_pid_gains(float kp, float ki, float kd);
    void update_vertical_velocity_pid_gains(float kp, float ki, float kd);
    void update_flying(uint8_t flying);
    void update_arming(uint8_t arm);
    void update_radio_failsafe(uint8_t radio_failsafe);
    void update_motor_emergency(uint8_t motor_emergency);
    void update_batt_failsafe(uint8_t batt_failsafe);

private:
    Logger()
    {
    }
    static Logger *_singleton;
    SdFat sd;
    FsFile logFile;
    RingBuf<FsFile, 10> ringBuffer;
    bool is_sd_card_inserted = false;
    LogEntry log_entry;
};

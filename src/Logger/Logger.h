#pragma once

#include <Arduino.h>
#include "SdFat.h"
#include "RingBuf.h"

#define SECTOR_SIZE 512

// Space to hold 96 ms of data for 128 byte lines at 250 sps.
#define RING_BUF_CAPACITY 6 * SECTOR_SIZE // 3, 072  bytes
#define LOG_SYNC 0xA55A
#define LOG_TYPE_PID_GAINS 1
#define LOG_TYPE_ENTRY 2

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

    struct __attribute__((packed)) LogPIDGains
    {
        uint16_t sync = LOG_SYNC;
        uint8_t type = LOG_TYPE_PID_GAINS;
        float roll_angle_kp = 0.0f;
        float roll_angle_ki = 0.0f;
        float roll_angle_kd = 0.0f;
        float pitch_angle_kp = 0.0f;
        float pitch_angle_ki = 0.0f;
        float pitch_angle_kd = 0.0f;
        float roll_rate_kp = 0.0f;
        float roll_rate_ki = 0.0f;
        float roll_rate_kd = 0.0f;
        float pitch_rate_kp = 0.0f;
        float pitch_rate_ki = 0.0f;
        float pitch_rate_kd = 0.0f;
        float yaw_rate_kp = 0.0f;
        float yaw_rate_ki = 0.0f;
        float yaw_rate_kd = 0.0f;
        float vertical_velocity_kp = 0.0f;
        float vertical_velocity_ki = 0.0f;
        float vertical_velocity_kd = 0.0f;
    };

    struct __attribute__((packed)) LogEntry
    {
        uint16_t sync = LOG_SYNC;
        uint8_t type = LOG_TYPE_ENTRY;
        uint32_t time_us = 0;
        uint16_t rc_throttle = 0;
        uint16_t rc_roll = 0;
        uint16_t rc_pitch = 0;
        uint16_t rc_yaw = 0;
        float desired_roll_angle = 0.0f;
        float desired_pitch_angle = 0.0f;
        float desired_roll_rate = 0.0f;
        float desired_pitch_rate = 0.0f;
        float desired_yaw_rate = 0.0f;
        float desired_vertical_velocity = 0.0f;
        float gyro_x = 0.0f;
        float gyro_y = 0.0f;
        float gyro_z = 0.0f;
        float acc_x = 0.0f;
        float acc_y = 0.0f;
        float acc_z = 0.0f;
        float estimated_roll_angle = 0.0f;
        float estimated_pitch_angle = 0.0f;
        float vertical_velocity = 0.0f;
        float altitude = 0.0f;
        float voltage = 0.0f;
        float current = 0.0f;
        float cmd_throttle = 0.0f;
        float cmd_roll = 0.0f;
        float cmd_pitch = 0.0f;
        float cmd_yaw = 0.0f;
        float cmd_hover = 0.0f;
        float m1 = 0.0f;
        float m2 = 0.0f;
        float m3 = 0.0f;
        float m4 = 0.0f;
        uint8_t is_flying = 0;
        uint8_t is_armed = 0;
        uint8_t is_radio_failsafe = 0;
        uint8_t is_motor_emergency = 0;
        uint8_t is_batt_failsafe = 0;
    };

    void init();
    void insert_log_pid_gains_to_buffer();
    void insert_log_entry_to_buffer();
    void write_logs_to_sd();
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
    RingBuf<FsFile, RING_BUF_CAPACITY> ringBuffer;
    bool is_sd_card_inserted = false;
    LogPIDGains log_pid_gains;
    LogEntry log_entry;
};

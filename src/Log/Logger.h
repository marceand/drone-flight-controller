#pragma once

#include <Arduino.h>

class Logger
{
public:
    static void update_throttle(uint16_t throttle);
    static void update_accelerometer(float ax, float ay, float az);
    static void update_vertical(float vz, float alt);
    static void update_voltage(float voltage);
    static void update_motors(float m1, float m2, float m3, float m4);
    static void update_commands(float roll, float pitch, float yaw, float throttle, float hover);
    static void update_flying(uint8_t flying);
    static void log_data();
    static void dump_logs();

    struct CurrentState
    {
        uint32_t t_ms = 0;
        uint16_t throttle = 0;
        float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
        float vertical_velocity = 0.0f;
        float altitude = 0.0f;
        float voltage = 0.0f;
        float m1 = 0.0f, m2 = 0.0f, m3 = 0.0f, m4 = 0.0f;
        float cmd_roll = 0.0f, cmd_pitch = 0.0f, cmd_yaw = 0.0f, cmd_throttle = 0.0f, cmd_hover = 0.0f;
        uint8_t is_flying = 0;
    };

    struct LogEntry
    {
        uint32_t t_ms;
        uint16_t throttle;
        float acc_x, acc_y, acc_z;
        float vertical_velocity;
        float altitude;
        float voltage;
        float m1, m2, m3, m4;
        float cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, cmd_hover;
        uint8_t is_flying;
    };

    static constexpr int MAX_LOGS = 7000;

private:
    static CurrentState current;
    static LogEntry logs[MAX_LOGS];
    static int logIndex;
};

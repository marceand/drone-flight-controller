#pragma once

#include <Arduino.h>
#include <SdFat.h>

#define LOG_BUFFER_SIZE 512 // Ring buffer entries
#define FLUSH_BATCH_SIZE 32 // Entries written per flush

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

    struct LogEntry
    {
        uint32_t time_us = 0;
        uint16_t throttle = 0;
        float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
        float vertical_velocity = 0.0f;
        float altitude = 0.0f;
        float voltage = 0.0f;
        float m1 = 0.0f, m2 = 0.0f, m3 = 0.0f, m4 = 0.0f;
        float cmd_roll = 0.0f, cmd_pitch = 0.0f, cmd_yaw = 0.0f, cmd_throttle = 0.0f, cmd_hover = 0.0f;
        uint8_t is_flying = 0;
    };

    void init();
    void update_logging();
    void flush_log_to_sd();
    void update_throttle(uint16_t throttle);
    void update_accelerometer(float ax, float ay, float az);
    void update_vertical(float vz, float alt);
    void update_voltage(float voltage);
    void update_motors(float m1, float m2, float m3, float m4);
    void update_commands(float roll, float pitch, float yaw, float throttle, float hover);
    void update_flying(uint8_t flying);
    // void log_data();
    // void dump_logs();

private:
    Logger() {}
    bool push_log(const LogEntry &entry);
    static Logger *_singleton;
    SdFat sd;
    FsFile logFile;
    LogEntry current;
    LogEntry logBuffer[LOG_BUFFER_SIZE];
    bool is_sd_card_inserted = false;
    volatile uint16_t head = 0;
    volatile uint16_t tail = 0;
};

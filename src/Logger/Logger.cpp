#include "Logger.h"
#define MAX_FILENAME 32

void Logger::init()
{
    // Serial.println(sizeof(LogEntry));
    is_sd_card_inserted = false;

    if (!sd.begin(SdioConfig(DMA_SDIO)))
    {
        return;
    }

    char filename[MAX_FILENAME];
    uint8_t index = 0;
    for (; index < 255; index++)
    {
        snprintf(filename, sizeof(filename), "flight%03u.bin", index);
        if (!sd.exists(filename))
        {
            break;
        }
    }

    if (index == 255)
    {
        return;
    }

    logFile = sd.open(filename, O_WRITE | O_CREAT | O_TRUNC);
    if (!logFile)
    {
        return;
    }

    is_sd_card_inserted = true;
}

bool Logger::push_log(const LogEntry &entry)
{
    uint16_t next = (head + 1) % LOG_BUFFER_SIZE;
    if (next == tail)
    {
        return false; // buffer full
    }
    logBuffer[head] = entry;
    head = next;
    return true;
}

void Logger::update_logging()
{

    if (!is_sd_card_inserted)
    {
        return;
    }

    LogEntry entry = log_entry;
    entry.time_us = micros();

    push_log(entry);
}

void Logger::flush_log_to_sd()
{

    if (!is_sd_card_inserted)
    {
        return;
    }

    LogEntry batch[FLUSH_BATCH_SIZE];
    size_t count = 0;

    while (tail != head && count < FLUSH_BATCH_SIZE)
    {
        batch[count++] = logBuffer[tail];
        tail = (tail + 1) % LOG_BUFFER_SIZE;
    }

    if (count > 0)
    {
        logFile.write((uint8_t *)batch, count * sizeof(LogEntry));
    }

    static uint8_t flushCounter = 0;
    if (++flushCounter >= 50)
    { // flush ~1 s
        logFile.flush();
        flushCounter = 0;
    }
}

void Logger::update_rc_inputs(uint16_t rc_throttle, uint16_t rc_roll, uint16_t rc_pitch, uint16_t rc_yaw)
{
    log_entry.rc_throttle = rc_throttle;
    log_entry.rc_roll = rc_roll;
    log_entry.rc_pitch = rc_pitch;
    log_entry.rc_yaw = rc_yaw;
}

void Logger::update_desired_angles(float desired_roll_angle, float desired_pitch_angle)
{
    log_entry.desired_roll_angle = desired_roll_angle;
    log_entry.desired_pitch_angle = desired_pitch_angle;
}

void Logger::update_desired_rates(float desired_yaw_rate)
{
    log_entry.desired_yaw_rate = desired_yaw_rate;
}

void Logger::update_desired_vertical_velocity(float desired_vertical_velocity)
{
    log_entry.desired_vertical_velocity = desired_vertical_velocity;
}

void Logger::update_gyro(float gx, float gy, float gz)
{
    log_entry.gyro_x = gx;
    log_entry.gyro_y = gy;
    log_entry.gyro_z = gz;
}

void Logger::update_accelerometer(float ax, float ay, float az)
{
    log_entry.acc_x = ax;
    log_entry.acc_y = ay;
    log_entry.acc_z = az;
}

void Logger::update_estimated_angles(float estimated_roll, float estimated_pitch)
{
    log_entry.estimated_roll_angle = estimated_roll;
    log_entry.estimated_pitch_angle = estimated_pitch;
}

void Logger::update_vertical(float vz, float alt)
{
    log_entry.vertical_velocity = vz;
    log_entry.altitude = alt;
}

void Logger::update_voltage_current(float voltage, float current)
{
    log_entry.voltage = voltage;
    log_entry.current = current;
}

void Logger::update_motors(float m1, float m2, float m3, float m4)
{
    log_entry.m1 = m1;
    log_entry.m2 = m2;
    log_entry.m3 = m3;
    log_entry.m4 = m4;
}

void Logger::update_commands(float throttle, float roll, float pitch, float yaw, float hover)
{
    log_entry.cmd_throttle = throttle;
    log_entry.cmd_roll = roll;
    log_entry.cmd_pitch = pitch;
    log_entry.cmd_yaw = yaw;
    log_entry.cmd_hover = hover;
}

void Logger::update_flying(uint8_t flying)
{
    log_entry.is_flying = flying;
}

void Logger::update_arming(uint8_t arm)
{
    log_entry.is_armed = arm;
}

void Logger::update_radio_failsafe(uint8_t radio_failsafe)
{
    log_entry.is_radio_failsafe = radio_failsafe;
}

void Logger::update_motor_emergency(uint8_t motor_emergency)
{
    log_entry.is_motor_emergency = motor_emergency;
}

#include "Logger.h"
#include <Entropy.h>

#define MAX_FILENAME 32

// Size to log 128 byte lines at 250Hz for ten minutes.
#define LOG_FILE_SIZE 128 * 250 * 600 // 19.2 megabytes.

void Logger::init()
{
    Entropy.Initialize();
    generate_session_id();

    // Serial.println(sizeof(LogPIDGains));
    // Serial.println(sizeof(LogEntry));

    // LogEntry e = log_entry; // some example data
    // uint8_t *ptr = (uint8_t *)&e;

    // Serial.print("LogEntry size: ");
    // Serial.println(sizeof(e));

    // for (size_t i = 0; i < sizeof(e); i++)
    // {
    //     Serial.print(ptr[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    is_sd_card_inserted = false;

    if (!sd.begin(SdioConfig(FIFO_SDIO)))
    {
        return;
    }

    // Create a file name of the format flight_log_000.bin
    char filename[MAX_FILENAME];
    uint8_t index = 0;
    for (; index < 255; index++)
    {
        snprintf(filename, sizeof(filename), "flight_log_%03u.bin", index);
        if (!sd.exists(filename))
        {
            break;
        }
    }

    if (index == 255)
    {
        return;
    }

    bool is_file_opened = logFile.open(filename, O_WRITE | O_CREAT | O_TRUNC);
    if (!is_file_opened)
    {
        return;
    }

    // File must be pre-allocated to avoid huge
    // delays searching for free clusters.
    bool is_file_preallocated = logFile.preAllocate(LOG_FILE_SIZE);
    if (!is_file_preallocated)
    {
        logFile.close();
        return;
    }

    ringBuffer.begin(&logFile);

    is_sd_card_inserted = true;
}

void Logger::generate_session_id()
{
    session_id = Entropy.random(0xFFFFFFFF); // Teensy built-in
    Serial.print("session-id: ");
    Serial.println(session_id);

    // if (session_id == 0)
    // {
    //     session_id = 1; // avoid zero if you want
    // }
}

void Logger::insert_log_pid_gains_to_buffer()
{
    if (!is_sd_card_inserted)
    {
        return;
    }

    LogPIDGains gains_entry = log_pid_gains;
    ringBuffer.write(&gains_entry, sizeof(gains_entry));
}

void Logger::insert_log_entry_to_buffer()
{

    // static uint8_t full_count = 0;

    if (!is_sd_card_inserted)
    {
        return;
    }

    LogEntry entry = log_entry;
    entry.time_us = micros();

    // LogEntry entry;

    // Make a safe, atomic copy of the struct
    // noInterrupts();
    // entry = log_entry;
    // interrupts();

    // Add the timestamp after the atomic copy
    // entry.time_us = micros();

    ringBuffer.write(&entry, sizeof(entry));

    // if (ringBuffer.getWriteError())
    // {
    // Serial.println("WriteError");
    // }

    // size_t count = ringBuffer.write(&entry, sizeof(entry));
    // if (count == 0)
    // {

    //     full_count++;
    //     Serial.println("Log buffer full count: ");
    //     Serial.println(full_count);
    // }
    // else
    // {
    //     if (full_count > 0)
    //     {
    //         full_count--;
    //         Serial.println("Log empty count: ");
    //         Serial.println(full_count);
    //     }
    // }
}

void Logger::write_logs_to_sd()
{

    if (!is_sd_card_inserted)
    {
        return;
    }

    size_t buffer_used_size = ringBuffer.bytesUsed();
    // Check if pre-allocated file is full
    if ((buffer_used_size + logFile.curPosition()) > (LOG_FILE_SIZE - 20))
    {
        // File is full
        return;
    }

    // If file not busy then allow writing one sector (512 bytes) before possible busy wait.

    // bool isBusy = logFile.isBusy();
    // if (isBusy)
    // {
    //     Serial.println("SD busy");

    // if (buffer_used_size >= SECTOR_SIZE)
    // {
    //     Serial.println("log not written");
    //     Serial.print("In Busy Buffer free size: ");
    //     Serial.println(ringBuffer.bytesFree());
    // }
    // }

    if (buffer_used_size >= SECTOR_SIZE && !logFile.isBusy())
    {
        // Write one sector (one sector is 512  bytes) from RingBuf to file.

        // uint32_t sd_write_time = micros();
        ringBuffer.writeOut(SECTOR_SIZE);
        // if (512 != ringBuffer.writeOut(SECTOR_SIZE))
        // {
        //     Serial.println("writeOut failed");
        // }

        // Serial.print("Written Buffer free size: ");
        // Serial.println(ringBuffer.bytesFree());
        // uint32_t diff = micros() - sd_write_time;
        // if (diff > 5)
        // {
        //     Serial.print("Big delay here: ");
        //     Serial.println(diff);
        // }
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

void Logger::update_desired_rates(float desired_roll_rate, float desired_pitch_rate, float desired_yaw_rate)
{
    log_entry.desired_roll_rate = desired_roll_rate;
    log_entry.desired_pitch_rate = desired_pitch_rate;
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

void Logger::update_commands(float throttle, float roll, float pitch, float yaw, float hover)
{
    log_entry.cmd_throttle = throttle;
    log_entry.cmd_roll = roll;
    log_entry.cmd_pitch = pitch;
    log_entry.cmd_yaw = yaw;
    log_entry.cmd_hover = hover;
}

void Logger::update_motors(float m1, float m2, float m3, float m4)
{
    log_entry.m1 = m1;
    log_entry.m2 = m2;
    log_entry.m3 = m3;
    log_entry.m4 = m4;
}

void Logger::update_roll_angle_pid_gains(float kp, float ki, float kd)
{
    log_pid_gains.roll_angle_kp = kp;
    log_pid_gains.roll_angle_ki = ki;
    log_pid_gains.roll_angle_kd = kd;
}

void Logger::update_pitch_angle_pid_gains(float kp, float ki, float kd)
{
    log_pid_gains.pitch_angle_kp = kp;
    log_pid_gains.pitch_angle_ki = ki;
    log_pid_gains.pitch_angle_kd = kd;
}

void Logger::update_roll_rate_pid_gains(float kp, float ki, float kd)
{
    log_pid_gains.roll_rate_kp = kp;
    log_pid_gains.roll_rate_ki = ki;
    log_pid_gains.roll_rate_kd = kd;
}

void Logger::update_pitch_rate_pid_gains(float kp, float ki, float kd)
{
    log_pid_gains.pitch_rate_kp = kp;
    log_pid_gains.pitch_rate_ki = ki;
    log_pid_gains.pitch_rate_kd = kd;
}

void Logger::update_yaw_rate_pid_gains(float kp, float ki, float kd)
{
    log_pid_gains.yaw_rate_kp = kp;
    log_pid_gains.yaw_rate_ki = ki;
    log_pid_gains.yaw_rate_kd = kd;
}

void Logger::update_vertical_velocity_pid_gains(float kp, float ki, float kd)
{
    log_pid_gains.vertical_velocity_kp = kp;
    log_pid_gains.vertical_velocity_ki = ki;
    log_pid_gains.vertical_velocity_kd = kd;
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

void Logger::update_batt_failsafe(uint8_t batt_failsafe)
{
    log_entry.is_batt_failsafe = batt_failsafe;
}

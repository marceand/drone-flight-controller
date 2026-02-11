#include "Logger.h"

#define MAX_FILENAME 32

// Size to log 132 byte lines at 250Hz for ten minutes.
#define LOG_FILE_SIZE 132 * 250 * 600 // 19.8 megabytes.

void Logger::init(uint32_t id) {

    // Serial.println(sizeof(LogEntry));
    set_session_id(id);

    is_sd_card_inserted = false;

    if (!sd.begin(SdioConfig(FIFO_SDIO)))
    {
        return;
    }

    // Create a file name of the format flight_log_001.bin
    char filename[MAX_FILENAME];
    uint8_t index = 1;
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

void Logger::set_session_id(uint32_t id)
{
    log_parameters.session_id = id;
    log_entry.session_id = id;
}

void Logger::insert_parameters_to_buffer()
{
    if (!is_sd_card_inserted)
    {
        return;
    }

    LogParameters parameters_entry = log_parameters;
    ringBuffer.write(&parameters_entry, sizeof(parameters_entry));
}

void Logger::insert_log_entry_to_buffer()
{
    if (!is_sd_card_inserted)
    {
        return;
    }

    LogEntry entry = log_entry;
    entry.time_us = micros();

    // if ring buffer is full, entry is not copy
    ringBuffer.write(&entry, sizeof(entry));
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
    if (buffer_used_size >= SECTOR_SIZE && !logFile.isBusy())
    {
        // Write one sector (one sector is 512  bytes) from RingBuf to file.
        ringBuffer.writeOut(SECTOR_SIZE);
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
    log_parameters.roll_angle_kp = kp;
    log_parameters.roll_angle_ki = ki;
    log_parameters.roll_angle_kd = kd;
}

void Logger::update_pitch_angle_pid_gains(float kp, float ki, float kd)
{
    log_parameters.pitch_angle_kp = kp;
    log_parameters.pitch_angle_ki = ki;
    log_parameters.pitch_angle_kd = kd;
}

void Logger::update_roll_rate_pid_gains(float kp, float ki, float kd)
{
    log_parameters.roll_rate_kp = kp;
    log_parameters.roll_rate_ki = ki;
    log_parameters.roll_rate_kd = kd;
}

void Logger::update_pitch_rate_pid_gains(float kp, float ki, float kd)
{
    log_parameters.pitch_rate_kp = kp;
    log_parameters.pitch_rate_ki = ki;
    log_parameters.pitch_rate_kd = kd;
}

void Logger::update_yaw_rate_pid_gains(float kp, float ki, float kd)
{
    log_parameters.yaw_rate_kp = kp;
    log_parameters.yaw_rate_ki = ki;
    log_parameters.yaw_rate_kd = kd;
}

void Logger::update_vertical_velocity_pid_gains(float kp, float ki, float kd)
{
    log_parameters.vertical_velocity_kp = kp;
    log_parameters.vertical_velocity_ki = ki;
    log_parameters.vertical_velocity_kd = kd;
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

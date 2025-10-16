#include "Logger.h"
#define MAX_FILENAME 32

void Logger::init()
{
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

    logFile = sd.open("flight_log.bin", O_WRITE | O_CREAT | O_TRUNC);
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

    LogEntry entry = current;
    entry.t_ms = micros();

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

void Logger::update_throttle(uint16_t throttle)
{
    current.throttle = throttle;
}

void Logger::update_accelerometer(float ax, float ay, float az)
{
    current.acc_x = ax;
    current.acc_y = ay;
    current.acc_z = az;
}

void Logger::update_vertical(float vz, float alt)
{
    current.vertical_velocity = vz;
    current.altitude = alt;
}

void Logger::update_voltage(float voltage)
{
    current.voltage = voltage;
}

void Logger::update_motors(float m1, float m2, float m3, float m4)
{
    current.m1 = m1;
    current.m2 = m2;
    current.m3 = m3;
    current.m4 = m4;
}

void Logger::update_commands(float roll, float pitch, float yaw, float throttle, float hover)
{
    current.cmd_roll = roll;
    current.cmd_pitch = pitch;
    current.cmd_yaw = yaw;
    current.cmd_throttle = throttle;
    current.cmd_hover = hover;
}

void Logger::update_flying(uint8_t flying)
{
    current.is_flying = flying;
}

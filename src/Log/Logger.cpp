#include "Logger.h"

Logger::CurrentState Logger::current;
DMAMEM Logger::LogEntry Logger::logs[Logger::MAX_LOGS];
int Logger::logIndex = 0;

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

// Log a snapshot
void Logger::log_data()
{
    if (logIndex < MAX_LOGS)
    {
        logs[logIndex++] = {
            micros(),
            current.throttle,
            current.acc_x,
            current.acc_y,
            current.acc_z,
            current.vertical_velocity,
            current.altitude,
            current.voltage,
            current.m1,
            current.m2,
            current.m3,
            current.m4,
            current.cmd_roll,
            current.cmd_pitch,
            current.cmd_yaw,
            current.cmd_throttle,
            current.cmd_hover,
            current.is_flying};
    }
}

// Dump to Serial
void Logger::dump_logs()
{
    for (int i = 0; i < logIndex; i++)
    {
        Serial.print("Time:");
        Serial.print(logs[i].t_ms);
        Serial.print("\t");
        Serial.print("Throttle:");
        Serial.print(logs[i].throttle);
        Serial.print("\t");
        Serial.print("AccX:");
        Serial.print(logs[i].acc_x);
        Serial.print("\t");
        Serial.print("AccY:");
        Serial.print(logs[i].acc_y);
        Serial.print("\t");
        Serial.print("AccZ:");
        Serial.print(logs[i].acc_z);
        Serial.print("\t");
        Serial.print("Vz:");
        Serial.print(logs[i].vertical_velocity);
        Serial.print("\t");
        Serial.print("Altitude:");
        Serial.print(logs[i].altitude);
        Serial.print("\t");
        Serial.print("voltage:");
        Serial.print(logs[i].voltage);
        Serial.print("\t");
        Serial.print("M1:");
        Serial.print(logs[i].m1);
        Serial.print("\t");
        Serial.print("M2:");
        Serial.print(logs[i].m2);
        Serial.print("\t");
        Serial.print("M3:");
        Serial.print(logs[i].m3);
        Serial.print("\t");
        Serial.print("M4:");
        Serial.print(logs[i].m4);
        Serial.print("\t");
        Serial.print("Roll_CMD:");
        Serial.print(logs[i].cmd_roll);
        Serial.print("\t");
        Serial.print("Pitch_CMD:");
        Serial.print(logs[i].cmd_pitch);
        Serial.print("\t");
        Serial.print("Yaw_CMD:");
        Serial.print(logs[i].cmd_yaw);
        Serial.print("\t");
        Serial.print("Throttle_CMD:");
        Serial.print(logs[i].cmd_throttle);
        Serial.print("\t");
        Serial.print("Hover_CMD:");
        Serial.print(logs[i].cmd_hover);
        Serial.print("\t");
        Serial.print("Flying:");
        Serial.println(logs[i].is_flying);
        delay(1);
        // Serial.printf("%u,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u\n",
        //               logs[i].t_ms,
        //               logs[i].roll, logs[i].pitch, logs[i].yaw, logs[i].throttle,
        //               logs[i].alt,
        //               logs[i].m1, logs[i].m2, logs[i].m3, logs[i].m4);
    }

    while (1)
    {
        /* code */
    }
}

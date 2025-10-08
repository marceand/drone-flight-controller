#pragma once

#include <Wire.h>
#include <Arduino.h>

#define GYRO_ACCEL_SLAVE_ADDRESS 0x68
#define GYRO_READING_REGISTER 0x43
#define GYRO_ACCEL_6_BYTE_READING 0x06
#define GYRO_POWER_MODE_REGISTER 0x6B
#define GYRO_POWER_MODE_VALUE 0x00
#define GYRO_LOW_PASS_FILTER_REGISTER 0x1A
#define GYRO_LOW_PASS_FILTER_10_HZ 0x05
#define GYRO_SENSITIVITY_REGISTER 0x1B
#define GYRO_SENSITIVITY_VALUE 0x08
#define GYRO_SENSITIVITY_SCALE 65.5
#define NUM_CALIBRATION_SAMPLES 2000
#define DESIRED_GYRO_FACTOR 0.15f
#define ACCEL_SENSITIVITY_REGISTER 0x1C
#define ACCEL_SENSITIVITY_VALUE 0x10
#define ACCEL_READING_REGISTER 0x3B
#define ACCEL_SENSITIVITY_SCALE 4096
#define GYRO_ACCEL_TEMP_14_BYTE_READING 14

class InertialSensor
{
public:
    InertialSensor(uint8_t address = GYRO_ACCEL_SLAVE_ADDRESS, TwoWire *wire = &Wire);
    void init();
    void read();
    float getRawGyroX() { return _gyroRaw.gyroX; }
    float getRawGyroY() { return _gyroRaw.gyroY; }
    float getRawGyroZ() { return _gyroRaw.gyroZ; }
    float getRawAccelX() { return _accelRaw.accelX; }
    float getRawAccelY() { return _accelRaw.accelY; }
    float getRawAccelZ() { return _accelRaw.accelZ; }
    float getCalibGyroX() { return _gyroRaw.gyroX - _gyroOffset.gyroX; }
    float getCalibGyroY() { return _gyroRaw.gyroY - _gyroOffset.gyroY; }
    float getCalibGyroZ() { return _gyroRaw.gyroZ - _gyroOffset.gyroZ; }
    float getCalibAccelX() { return _accelRaw.accelX - _accelOffset.accelX; }
    float getCalibAccelY() { return _accelRaw.accelY - _accelOffset.accelY; }
    float getCalibAccelZ() { return _accelRaw.accelZ - _accelOffset.accelZ; }
    float getTemperature() { return _temperature; }

private:
    struct gyro_t
    {
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    struct accel_t
    {
        float accelX;
        float accelY;
        float accelZ;
    };

    uint8_t _address;
    TwoWire *_wire;
    gyro_t _gyroRaw = {0.0, 0.0, 0.0};
    gyro_t _gyroOffset = {0.0, 0.0, 0.0};
    accel_t _accelRaw = {0.0, 0.0, 0.0};
    accel_t _accelOffset = {0.0, 0.0, 0.0};
    float _temperature = 0.0;
    void startGyroPowerMode(void);
    void setGyroLowPassFilter(void);
    void setGyroSensitivity(void);
    void setAccelSensitivity(void);
    void calculateGyroOffset(void);
    void calculateAccelOffset(void);
    InertialSensor::gyro_t readRawGyro(void);
    InertialSensor::accel_t readRawAccel(void);
    float scaleGyroReading(float reading);
    float scaleAccelReading(float reading);
};

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
#define GYRO_CALIBRATION_COUNT 2000
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
    float getRawGyroX() { return _rawGyro.gyroX; }
    float getRawGyroY() { return _rawGyro.gyroY; }
    float getRawGyroZ() { return _rawGyro.gyroZ; }
    float getRawAccelX() { return _rawAccel.accelX; }
    float getRawAccelY() { return _rawAccel.accelY; }
    float getRawAccelZ() { return _rawAccel.accelZ; }
    float getCalibGyroX() { return _rawGyro.gyroX - _gyroCalib.gyroX; }
    float getCalibGyroY() { return _rawGyro.gyroY - _gyroCalib.gyroY; }
    float getCalibGyroZ() { return _rawGyro.gyroZ - _gyroCalib.gyroZ; }
    float getCalibAccelX() { return _rawAccel.accelX; }
    float getCalibAccelY() { return _rawAccel.accelY; }
    float getCalibAccelZ() { return _rawAccel.accelZ; }
    float getTemperature() { return _temperature; }
    float getRollAngle() { return _roll_angle; }
    float getPitchAngle() { return _pitch_angle; }

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
    gyro_t _rawGyro = {0.0, 0.0, 0.0};
    gyro_t _gyroCalib = {0.0, 0.0, 0.0};
    accel_t _rawAccel = {0.0, 0.0, 0.0};
    float _temperature = 0.0;
    float _roll_angle = 0.0;
    float _pitch_angle = 0.0;
    void startGyroPowerMode(void);
    void setGyroLowPassFilter(void);
    void setGyroSensitivity(void);
    void setAccelSensitivity(void);
    void calibrateGyro(void);
    InertialSensor::gyro_t readRawGyro(void);
    InertialSensor::accel_t readRawAccel(void);
    float scaleGyroReading(float reading);
    float scaleAccelReading(float reading);
    void calculateAngles(void);
};

#include "InertialSensor.h"

InertialSensor::InertialSensor(uint8_t address, TwoWire *wire)
{
    _address = address;
    _wire = wire;
}

void InertialSensor::init()
{
    startGyroPowerMode();
    setGyroLowPassFilter();
    setAccelSensitivity();
    setGyroSensitivity();
    calibrateGyro();
}

void InertialSensor::read()
{
    _wire->beginTransmission(_address);
    _wire->write(ACCEL_READING_REGISTER); // start at  acceleromet register for 14 bytes reading
    _wire->endTransmission();

    _wire->requestFrom(_address, GYRO_ACCEL_TEMP_14_BYTE_READING);

    int16_t accelX = _wire->read() << 8 | _wire->read();
    int16_t accelY = _wire->read() << 8 | _wire->read();
    int16_t accelZ = _wire->read() << 8 | _wire->read();

    int16_t temperature = _wire->read() << 8 | _wire->read();

    int16_t gyroX = _wire->read() << 8 | _wire->read();
    int16_t gyroY = _wire->read() << 8 | _wire->read();
    int16_t gyroZ = _wire->read() << 8 | _wire->read();

    _rawAccel.accelX = scaleAccelReading((float)accelX);
    _rawAccel.accelY = scaleAccelReading((float)accelY);
    _rawAccel.accelZ = scaleAccelReading((float)accelZ);

    _temperature = (float)temperature;

    _rawGyro.gyroX = scaleGyroReading((float)gyroX);
    _rawGyro.gyroY = scaleGyroReading((float)gyroY);
    _rawGyro.gyroZ = scaleGyroReading((float)gyroZ);

    calculateAngles();
}

void InertialSensor::startGyroPowerMode(void)
{
    _wire->beginTransmission(_address);
    _wire->write(GYRO_POWER_MODE_REGISTER);
    _wire->write(GYRO_POWER_MODE_VALUE);
    _wire->endTransmission();
}

void InertialSensor::setGyroLowPassFilter(void)
{
    _wire->beginTransmission(_address);
    _wire->write(GYRO_LOW_PASS_FILTER_REGISTER);
    _wire->write(GYRO_LOW_PASS_FILTER_10_HZ);
    _wire->endTransmission();
}

void InertialSensor::setGyroSensitivity(void)
{
    _wire->beginTransmission(_address);
    _wire->write(GYRO_SENSITIVITY_REGISTER);
    _wire->write(GYRO_SENSITIVITY_VALUE);
    _wire->endTransmission();
}

void InertialSensor::setAccelSensitivity(void)
{
    _wire->beginTransmission(_address);
    _wire->write(ACCEL_SENSITIVITY_REGISTER);
    _wire->write(ACCEL_SENSITIVITY_VALUE);
    _wire->endTransmission();
}

void InertialSensor::calibrateGyro(void)
{
    for (int n = 1; n <= GYRO_CALIBRATION_COUNT; n++)
    {
        gyro_t rawRate = readRawGyro();
        _gyroCalib.gyroX += rawRate.gyroX;
        _gyroCalib.gyroY += rawRate.gyroY;
        _gyroCalib.gyroZ += rawRate.gyroZ;
        delay(1);
    }

    _gyroCalib.gyroX /= GYRO_CALIBRATION_COUNT;
    _gyroCalib.gyroY /= GYRO_CALIBRATION_COUNT;
    _gyroCalib.gyroZ /= GYRO_CALIBRATION_COUNT;
}

InertialSensor::gyro_t InertialSensor::readRawGyro(void)
{
    gyro_t rawRate;

    _wire->beginTransmission(_address);
    _wire->write(GYRO_READING_REGISTER);
    _wire->endTransmission();

    _wire->requestFrom(_address, GYRO_ACCEL_6_BYTE_READING);

    int16_t gyroX = _wire->read() << 8 | _wire->read();
    int16_t gyroY = _wire->read() << 8 | _wire->read();
    int16_t gyroZ = _wire->read() << 8 | _wire->read();

    rawRate.gyroX = scaleGyroReading((float)gyroX);
    rawRate.gyroY = scaleGyroReading((float)gyroY);
    rawRate.gyroZ = scaleGyroReading((float)gyroZ);

    return rawRate;
}

InertialSensor::accel_t InertialSensor::readRawAccel(void)
{
    accel_t rawAccel;

    _wire->beginTransmission(_address);
    _wire->write(ACCEL_READING_REGISTER);
    _wire->endTransmission();

    _wire->requestFrom(_address, GYRO_ACCEL_6_BYTE_READING);

    int16_t accelX = _wire->read() << 8 | _wire->read();
    int16_t accelY = _wire->read() << 8 | _wire->read();
    int16_t accelZ = _wire->read() << 8 | _wire->read();

    rawAccel.accelX = scaleAccelReading((float)accelX);
    rawAccel.accelY = scaleAccelReading((float)accelY);
    rawAccel.accelZ = scaleAccelReading((float)accelZ);

    return rawAccel;
}

float InertialSensor::scaleGyroReading(float reading)
{
    return reading / GYRO_SENSITIVITY_SCALE;
}

float InertialSensor::scaleAccelReading(float reading)
{
    return reading / ACCEL_SENSITIVITY_SCALE;
}

void InertialSensor::calculateAngles(void)
{
    _roll_angle = atan(_rawAccel.accelY / sqrt(_rawAccel.accelX * _rawAccel.accelX + _rawAccel.accelZ * _rawAccel.accelZ)) * 1 / (3.142 / 180);
    _pitch_angle = -atan(_rawAccel.accelX / sqrt(_rawAccel.accelY * _rawAccel.accelY + _rawAccel.accelZ * _rawAccel.accelZ)) * 1 / (3.142 / 180);
}

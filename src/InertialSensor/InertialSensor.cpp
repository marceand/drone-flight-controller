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
    calculateGyroOffset();
    calculateAccelOffset();
}

void InertialSensor::read()
{
    _wire->beginTransmission(_address);
    _wire->write(ACCEL_READING_REGISTER);
    _wire->endTransmission();

    _wire->requestFrom(_address, GYRO_ACCEL_TEMP_14_BYTE_READING);

    int16_t accelX = _wire->read() << 8 | _wire->read();
    int16_t accelY = _wire->read() << 8 | _wire->read();
    int16_t accelZ = _wire->read() << 8 | _wire->read();

    int16_t temperature = _wire->read() << 8 | _wire->read();

    int16_t gyroX = _wire->read() << 8 | _wire->read();
    int16_t gyroY = _wire->read() << 8 | _wire->read();
    int16_t gyroZ = _wire->read() << 8 | _wire->read();

    _accelRaw.accelX = scaleAccelReading((float)accelX);
    _accelRaw.accelY = scaleAccelReading((float)accelY);
    _accelRaw.accelZ = scaleAccelReading((float)accelZ);

    _temperature = (float)temperature;

    _gyroRaw.gyroX = scaleGyroReading((float)gyroX);
    _gyroRaw.gyroY = scaleGyroReading((float)gyroY);
    _gyroRaw.gyroZ = scaleGyroReading((float)gyroZ);

    calculateAngles();
    calculateVerticalAcceleration();
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

void InertialSensor::calculateGyroOffset(void)
{
    for (int n = 1; n <= NUM_CALIBRATION_SAMPLES; n++)
    {
        gyro_t rawRate = readRawGyro();
        _gyroOffset.gyroX += rawRate.gyroX;
        _gyroOffset.gyroY += rawRate.gyroY;
        _gyroOffset.gyroZ += rawRate.gyroZ;
        delay(1);
    }

    _gyroOffset.gyroX /= NUM_CALIBRATION_SAMPLES;
    _gyroOffset.gyroY /= NUM_CALIBRATION_SAMPLES;
    _gyroOffset.gyroZ /= NUM_CALIBRATION_SAMPLES;
}

void InertialSensor::calculateAccelOffset(void)
{
    for (int n = 1; n <= NUM_CALIBRATION_SAMPLES; n++)
    {
        accel_t rawAccel = readRawAccel();
        _accelOffset.accelX += rawAccel.accelX;
        _accelOffset.accelY += rawAccel.accelY;
        _accelOffset.accelZ += rawAccel.accelZ;
        delay(1);
    }

    _accelOffset.accelX /= NUM_CALIBRATION_SAMPLES;
    _accelOffset.accelY /= NUM_CALIBRATION_SAMPLES;
    _accelOffset.accelZ /= NUM_CALIBRATION_SAMPLES;

    _accelOffset.accelZ -= 1.00;
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
    float accelX = getCalibAccelX();
    float accelY = getCalibAccelY();
    float accelZ = getCalibAccelZ();
    _roll_angle = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * 1 / (3.142 / 180);
    _pitch_angle = -atan(accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 1 / (3.142 / 180);
}

void InertialSensor::calculateVerticalAcceleration()
{
    float accelX = getCalibAccelX();
    float accelY = getCalibAccelY();
    float accelZ = getCalibAccelZ();
    float accel_z_inertial = -sin(_pitch_angle * (3.142 / 180)) * accelX +
                             cos(_pitch_angle * (3.142 / 180)) * sin(_roll_angle * (3.142 / 180)) * accelY +
                             cos(_pitch_angle * (3.142 / 180)) * cos(_roll_angle * (3.142 / 180)) * accelZ;

    _vertical_acceleration = (accel_z_inertial - 1.0) * 9.81 * 100; // cm/s^2
}

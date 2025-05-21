#include "Barometer_BMP280.h"

#define BMP280_ADDRESS 0x76
#define BMP280_DATA_REGISTER 0xF7
#define BMP280_CTRL_MEAS_REGISTER 0xF4
#define BMP280_CTRL_MEAS_INDOOR_VALUE 0x57
#define BMP280_CONFIG_REGISTER 0xF5
#define BMP280_CONFIG_VALUE 0x14
#define BMP280_CALIB_REGISTER 0x88
#define BMP280_CALIB_24_BYTE_READING 24
#define BMP280_DATA_6_BYTE_READING 6

void Barometer_BMP280::init()
{
    setControlMeasurement();
    setConfig();
    readCalibrationData();
    calculate_reference_altitude();
}

void Barometer_BMP280::read()
{
    _wire->beginTransmission(BMP280_ADDRESS);
    _wire->write(BMP280_DATA_REGISTER);
    _wire->endTransmission();
    _wire->requestFrom(BMP280_ADDRESS, BMP280_DATA_6_BYTE_READING);

    uint32_t press_msb = _wire->read();
    uint32_t press_lsb = _wire->read();
    uint32_t press_xlsb = _wire->read();
    uint32_t temp_msb = _wire->read();
    uint32_t temp_lsb = _wire->read();
    uint32_t temp_xlsb = _wire->read();

    int32_t adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    int32_t adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

    updateTemperature(adc_T);
    updatePressure(adc_P);
}

void Barometer_BMP280::setControlMeasurement()
{
    _wire->beginTransmission(BMP280_ADDRESS);
    _wire->write(BMP280_CTRL_MEAS_REGISTER);
    _wire->write(BMP280_CTRL_MEAS_INDOOR_VALUE);
    _wire->endTransmission();
}

void Barometer_BMP280::setConfig()
{
    _wire->beginTransmission(BMP280_ADDRESS);
    _wire->write(BMP280_CONFIG_REGISTER);
    _wire->write(BMP280_CONFIG_VALUE);
    _wire->endTransmission();
}

void Barometer_BMP280::readCalibrationData()
{
    _wire->beginTransmission(BMP280_ADDRESS);
    _wire->write(BMP280_CALIB_REGISTER);
    _wire->endTransmission();

    _wire->requestFrom(BMP280_ADDRESS, BMP280_CALIB_24_BYTE_READING);

    uint8_t buf[24];
    uint8_t i = 0;

    while (_wire->available())
    {
        buf[i] = _wire->read();
        i++;
    }

    dig_T1 = (buf[1] << 8) | buf[0];
    dig_T2 = (buf[3] << 8) | buf[2];
    dig_T3 = (buf[5] << 8) | buf[4];
    dig_P1 = (buf[7] << 8) | buf[6];
    dig_P2 = (buf[9] << 8) | buf[8];
    dig_P3 = (buf[11] << 8) | buf[10];
    dig_P4 = (buf[13] << 8) | buf[12];
    dig_P5 = (buf[15] << 8) | buf[14];
    dig_P6 = (buf[17] << 8) | buf[16];
    dig_P7 = (buf[19] << 8) | buf[18];
    dig_P8 = (buf[21] << 8) | buf[20];
    dig_P9 = (buf[23] << 8) | buf[22];

    delay(250);
}

void Barometer_BMP280::updateTemperature(int32_t adc_T)
{
    int32_t var1, var2, t;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    _t_fine = var1 + var2;
    t = (_t_fine * 5 + 128) >> 8;

    _temperature = ((float)t) * 0.01f;
}

void Barometer_BMP280::updatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0)
    {
        return;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    _pressure = (float)p / 256.0f;
}

void Barometer_BMP280::calculate_reference_altitude()
{
    int reading_length = 2000;
    float altitude_sum = 0.0;
    for (int i = 0; i < reading_length; i++)
    {
        read();
        altitude_sum = altitude_sum + get_pressure_altitude_in_cm();
        delay(1);
    }

    _reference_altitude = altitude_sum / reading_length;
}

float Barometer_BMP280::calculate_pressure_altitude(float pressure_in_hPa)
{
    return 44330 * (1 - pow(pressure_in_hPa / 1013.25, 1 / 5.255));
}

float Barometer_BMP280::get_pressure_altitude_in_cm()
{
    float pressure_in_hPa = _pressure / 100.0f;                   // convert to hPa
    return calculate_pressure_altitude(pressure_in_hPa) * 100.0f; // convert to cm
}

float Barometer_BMP280::get_relative_altitude_in_cm()
{
    return get_pressure_altitude_in_cm() - get_reference_altitude_in_cm();
}

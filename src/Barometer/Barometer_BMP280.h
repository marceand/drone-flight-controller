#pragma once

#include <Wire.h>

class Barometer_BMP280
{
    /* data */
public:
    Barometer_BMP280(TwoWire *wire = &Wire) : _wire(wire) {}
    void init();
    void read();
    float get_reference_altitude_in_cm()
    {
        return _reference_altitude;
    }
    float get_pressure_altitude_in_cm();
    float get_relative_altitude_in_cm();

private:
    TwoWire *_wire;
    float _temperature;
    float _pressure; // In Pascal
    float _reference_altitude;
    int32_t _t_fine;
    uint16_t dig_T1, dig_P1;
    int16_t dig_T2, dig_T3;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    void setControlMeasurement();
    void setConfig();
    void readCalibrationData();
    void updateTemperature(int32_t adc_T);
    void updatePressure(int32_t adc_P);
    void calculate_reference_altitude();
    float calculate_pressure_altitude(float pressure_in_hPa);
};

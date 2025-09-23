#pragma once

#include "../HAL/BatteryReader.h"

class BatteryMonitor
{
public:
    void init();
    void monitor();
    float initial_capacity()
    {
        return _batt_capacity_initial;
    }
    float voltage()
    {
        return _voltage;
    }

    float current()
    {
        return _current;
    }
    float get_remaining_percentage()
    {
        return _batt_remaining_percentage;
    }
    bool is_batt_failsafe()
    {
        return _is_batt_failsafe;
    }

private:
    BatteryReader _batteryReader;
    float _voltage = 0.0f;
    float _current = 0.0f;
    float _current_consumed = 0.0f;
    float _batt_remaining_percentage = 0.0f;
    const float _batt_capacity_default = 1300.0f;
    float _batt_capacity_initial = 0.0f;
    bool _is_batt_failsafe = false;
    float calculateBatteryCapacity(float voltage);
};

#pragma once

#include "../HAL/BatteryReader.h"
#include "../HAL/LEDIndicator.h"

class BatteryMonitor
{
public:
    BatteryMonitor(LEDIndicator &led) : _LEDIndicator(led) {}
    void init();
    void monitor();

private:
    BatteryReader _batteryReader;
    LEDIndicator &_LEDIndicator;
    float _current_consumed = 0.0f;
    float _batt_remaining_percentage;
    float _batt_capacity_default = 1300.0f;
    float _batt_capacity_initial;
    float calculateBatteryCapacity(float voltage);
};

#include "BatteryMonitor.h"

void BatteryMonitor::init()
{
    _LEDIndicator.init();

    float voltage = _batteryReader.voltage();
    _batt_capacity_initial = calculateBatteryCapacity(voltage);

    if (voltage < 7.5f)
    {
        _LEDIndicator.enableRedLED();
    }
    else
    {
        _LEDIndicator.disableRedLED();
    }
}

void BatteryMonitor::monitor()
{
    _voltage = _batteryReader.voltage();
    _current = _batteryReader.current();
    _current_consumed = _current * (1000.0f / 3600.0f) * 0.004f + _current_consumed;
    _batt_remaining_percentage = ((_batt_capacity_initial - _current_consumed) / _batt_capacity_default) * 100.0f;

    if (_batt_remaining_percentage <= 30.0f)
    {
        _LEDIndicator.enableRedLED();
    }
    else
    {
        _LEDIndicator.disableRedLED();
    }
}

float BatteryMonitor::calculateBatteryCapacity(float voltage)
{
    if (voltage > 8.3f)
    {
        return _batt_capacity_default;
    }
    else if (voltage < 7.5f)
    {
        return (30.0f / 100.0f) * _batt_capacity_default;
    }
    else
    {
        return ((82.0f * voltage - 580.0f) / 100.0f) * _batt_capacity_default;
    }
}

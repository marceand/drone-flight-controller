#include "BatteryMonitor.h"
#include "../Notify/StatusNotifier.h"

void BatteryMonitor::init()
{
    float voltage = _batteryReader.voltage();
    _batt_capacity_initial = calculateBatteryCapacity(voltage);

    if (voltage < 7.5f)
    {
        StatusNotifier::events.failsafe_battery = true;
    }
    else
    {
        StatusNotifier::events.failsafe_battery = false;
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
        _is_batt_failsafe = true;
        StatusNotifier::events.failsafe_battery = true;
    }
    else
    {
        _is_batt_failsafe = false;
        StatusNotifier::events.failsafe_battery = false;
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

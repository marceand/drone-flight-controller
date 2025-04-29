#include "BatteryMonitor.h"
#include <Arduino.h>

void BatteryMonitor::init()
{
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);

    readVoltage();

    if (_voltage > 8.3)
    {
        digitalWrite(5, LOW);
        _batt_capacity_initial = _batt_capacity_default;
    }
    else if (_voltage < 7.5)
    {
        _batt_capacity_initial = 30 / 100 * _batt_capacity_default;
    }
    else
    {
        digitalWrite(5, LOW);
        _batt_capacity_initial = (82 * _voltage - 580) / 100 * _batt_capacity_default;
    }
}

void BatteryMonitor::monitor()
{
    readVoltage();
    readCurrent();

    _current_consumed = _current * 1000 * 0.004 / 3600 + _current_consumed;
    _batt_remaining_percentage = (_batt_capacity_initial - _current_consumed) / _batt_capacity_default * 100;

    if (_batt_remaining_percentage <= 30)
    {
        digitalWrite(5, HIGH);
    }
    else
    {
        digitalWrite(5, LOW);
    }
}

void BatteryMonitor::readVoltage()
{
    _voltage = (float)analogRead(15) / 62;
}

void BatteryMonitor::readCurrent()
{
    _current = (float)analogRead(21) / 0.089;
}

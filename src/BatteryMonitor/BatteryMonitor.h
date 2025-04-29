#pragma once

class BatteryMonitor
{
public:
    void init();
    void monitor();

private:
    float _voltage;
    float _current;
    float _current_consumed = 0.0f;
    float _batt_remaining_percentage;
    float _batt_capacity_default = 1300.0f;
    float _batt_capacity_initial;

    void readVoltage();
    void readCurrent();
};

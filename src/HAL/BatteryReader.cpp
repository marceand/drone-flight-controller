#include "BatteryReader.h"
#include <Arduino.h>

#define VOLTAGE_PIN 15
#define CURRENT_PIN 21

float BatteryReader::voltage()
{
    return (float)analogRead(VOLTAGE_PIN) / 62;
}

float BatteryReader::current()
{
    return (float)analogRead(CURRENT_PIN) * 0.089;
}

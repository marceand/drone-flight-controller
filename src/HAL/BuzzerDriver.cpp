#include "BuzzerDriver.h"
#include <Arduino.h>

#define BUZZER_PIN 1

void BuzzerDriver::init()
{
    if (is_initialized)
    {
        return;
    }

    is_initialized = true;
    pinMode(BUZZER_PIN, OUTPUT);
    disableTone();
}

void BuzzerDriver::enableTone()
{
    digitalWrite(BUZZER_PIN, HIGH);
}

void BuzzerDriver::disableTone()
{
    digitalWrite(BUZZER_PIN, LOW);
}

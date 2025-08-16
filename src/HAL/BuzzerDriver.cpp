#include "BuzzerDriver.h"
#include <Arduino.h>

#define BUZZER_PIN 23

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
    digitalWriteFast(BUZZER_PIN, HIGH);
}

void BuzzerDriver::disableTone()
{
    digitalWriteFast(BUZZER_PIN, LOW);
}

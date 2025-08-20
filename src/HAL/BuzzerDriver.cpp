#include "BuzzerDriver.h"

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

void BuzzerDriver::start_tone(uint16_t frequency)
{
    analogWriteFrequency(BUZZER_PIN, frequency);
    analogWrite(BUZZER_PIN, 128); // 50% duty
}

void BuzzerDriver::stop_tone()
{
    analogWrite(BUZZER_PIN, 0); // 50% duty
}

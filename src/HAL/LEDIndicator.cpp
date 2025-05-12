#include "LEDIndicator.h"
#include <Arduino.h>

#define RED_LED_PIN 5
#define GREEN_LED_PIN 6

void LEDIndicator::init()
{
    if (is_initialized)
    {
        return;
    }
    is_initialized = true;

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
}

void LEDIndicator::enableRedLED()
{
    digitalWrite(RED_LED_PIN, HIGH);
}

void LEDIndicator::disableRedLED()
{
    digitalWrite(RED_LED_PIN, LOW);
}

void LEDIndicator::enableGreenLED()
{
    digitalWrite(GREEN_LED_PIN, HIGH);
}

void LEDIndicator::disableGreenLED()
{
    digitalWrite(GREEN_LED_PIN, LOW);
}

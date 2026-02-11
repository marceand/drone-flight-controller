#include "LEDDriver.h"
#include <Arduino.h>

#define RED_LED_PIN 5
#define GREEN_LED_PIN 6

void LEDDriver::init()
{
    if (is_initialized)
    {
        return;
    }
    is_initialized = true;

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    enableGreenLED();
    enableRedLED();
    delay(1000);
    disableGreenLED();
    disableRedLED();
}

void LEDDriver::enableRedLED()
{
    digitalWrite(RED_LED_PIN, HIGH);
}

void LEDDriver::disableRedLED()
{
    digitalWrite(RED_LED_PIN, LOW);
}

void LEDDriver::enableGreenLED()
{
    digitalWrite(GREEN_LED_PIN, HIGH);
}

void LEDDriver::disableGreenLED()
{
    digitalWrite(GREEN_LED_PIN, LOW);
}

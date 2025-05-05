#include "ESCOutput.h"
#include <Arduino.h>

void ESCOutput::inits()
{
    if (is_initalized)
    {
        return;
    }
    is_initalized = true;
    analogWriteFrequency(MOTOR_1_PIN, 250);
    analogWriteFrequency(MOTOR_2_PIN, 250);
    analogWriteFrequency(MOTOR_3_PIN, 250);
    analogWriteFrequency(MOTOR_4_PIN, 250);
    analogWriteResolution(12);
}

void ESCOutput::update_motor_1_speed(float input)
{
    analogWrite(MOTOR_1_PIN, input);
}

void ESCOutput::update_motor_2_speed(float input)
{
    analogWrite(MOTOR_2_PIN, input);
}

void ESCOutput::update_motor_3_speed(float input)
{
    analogWrite(MOTOR_3_PIN, input);
}

void ESCOutput::update_motor_4_speed(float input)
{
    analogWrite(MOTOR_4_PIN, input);
}

#include "ESCOutput.h"
#include <Arduino.h>

void ESCOutput::init()
{
    if (is_initalized)
    {
        return;
    }
    is_initalized = true;

    for (int i = 0; i < NUM_ESC_CHANNELS; i++)
    {
        analogWriteFrequency(escChannels[i].pin, 250);
    }
    analogWriteResolution(12);
}

float ESCOutput::scale_ouput(float pwm)
{
    return SCALE_TO_12_BIT * pwm;
}

void ESCOutput::set_pwm_value(int index, float scale_pwm)
{
    if (index >= 0 && index < NUM_ESC_CHANNELS)
    {
        escChannels[index].pwm_value = scale_pwm;
    }
}

void ESCOutput::write_pwm_outputs()
{
    for (int i = 0; i < NUM_ESC_CHANNELS; i++)
    {
        analogWrite(escChannels[i].pin, escChannels[i].pwm_value);
    }
}
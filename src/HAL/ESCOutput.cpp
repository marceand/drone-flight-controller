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

    // M1.attach(1, 1000, 2000);
    // M2.attach(2, 1000, 2000);
    // M3.attach(3, 1000, 2000);
    // M4.attach(4, 1000, 2000);
}

float ESCOutput::scale_ouput(float pwm)
{
    return SCALE_TO_12_BIT * pwm;
}

void ESCOutput::set_pwm_value(int index, float pwm)
{
    if (index >= 0 && index < NUM_ESC_CHANNELS)
    {
        escChannels[index].pwm_value = pwm;
        escChannels[index].scaled_pwm_value = scale_ouput(pwm);
    }
}

void ESCOutput::write_pwm_outputs()
{

    // M1.write(us_to_deg(escChannels[0].pwm_value));
    // M2.write(us_to_deg(escChannels[1].pwm_value));
    // M3.write(us_to_deg(escChannels[2].pwm_value));
    // M4.write(us_to_deg(escChannels[3].pwm_value));

    for (int i = 0; i < NUM_ESC_CHANNELS; i++)
    {
        analogWrite(escChannels[i].pin, escChannels[i].scaled_pwm_value);
    }

    // Serial.print("\t");
    // Serial.print("M1:");
    // Serial.print(escChannels[0].pwm_value);
    // Serial.print("\t");
    // Serial.print("M2:");
    // Serial.print(escChannels[1].pwm_value);
    // Serial.print("\t");
    // Serial.print("M3:");
    // Serial.print(escChannels[2].pwm_value);
    // Serial.print("\t");
    // Serial.print("M4:");
    // Serial.println(escChannels[3].pwm_value);
}

int ESCOutput::us_to_deg(int us)
{
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    return (us - 1000) * 0.18f; // convert to 0–180 degrees
}
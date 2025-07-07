#pragma once

#define ESC_1_PIN 1
#define ESC_2_PIN 2
#define ESC_3_PIN 3
#define ESC_4_PIN 4
#define SCALE_TO_12_BIT 1.024f
#define NUM_ESC_CHANNELS 4

class ESCOutput
{
public:
    struct ESCChannel
    {
        uint8_t pin;
        float pwm_value;
    };
    void init();
    float scale_ouput(float pwm);
    void write(int index, float scale_pwm);
    void push();

private:
    bool is_initalized = false;
    ESCChannel escChannels[NUM_ESC_CHANNELS] = {
        {ESC_1_PIN, 1000.0f},
        {ESC_2_PIN, 1000.0f},
        {ESC_3_PIN, 1000.0f},
        {ESC_4_PIN, 1000.0f}};
};

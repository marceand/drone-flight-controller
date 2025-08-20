#pragma once

#include <Arduino.h>

class BuzzerDriver
{
public:
    void init();
    void enableTone();
    void disableTone();
    void start_tone(uint16_t frequency);
    void stop_tone();

private:
    bool is_initialized = false;
};

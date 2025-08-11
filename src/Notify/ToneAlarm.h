#pragma once
#include <cstdint>
#include "../HAL/BuzzerDriver.h"
#include <Arduino.h>

class ToneAlarm
{
public:
    ToneAlarm(BuzzerDriver &buzzer);

    struct Note
    {
        uint16_t frequency;
        uint16_t duration;
    };

    enum ToneID
    {
        TONE_NONE,
        TONE_STARTUP,
        TONE_ARMING,
        TONE_DISARMING,
        TONE_FAILSAFE,
        TONE_COUNT
    };

    struct Tone
    {
        const ToneID id;
        Note notes[21];
        uint8_t length;
    };

    void init();
    void update();
    void play_tone(ToneID id);
    void stop_tone();

private:
    BuzzerDriver _buzzer;
    ToneID _current_tone_id;
    uint8_t _tone_index;
    uint32_t _elapsed_tone_time;
    uint32_t _elapsed_half_period;
    uint32_t _half_period;
    uint32_t _last_update_time;
    bool _tone_state;

    static const Tone _tones[TONE_COUNT];

    uint32_t calculate_half_period(uint16_t frequency);
};

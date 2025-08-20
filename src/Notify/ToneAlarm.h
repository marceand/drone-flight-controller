#pragma once
#include <cstdint>
#include "../HAL/BuzzerDriver.h"
#include <Arduino.h>
#include <IntervalTimer.h>

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
        TONE_LOW_BATT,
        TONE_FAILSAFE_RADIO,
        TONE_COUNT
    };

    struct Tone
    {
        const ToneID id;
        const Note *notes;
        uint8_t length;
        bool continuous;
    };

    struct ToneFlags
    {
        bool startup = false;
        bool armed = false;
        bool failsafe_radio = false;
        bool failsafe_battery = false;
    };

    static ToneFlags events;

    void init();
    void update();
    void stop_tone();

private:
    IntervalTimer _toneTimer;
    BuzzerDriver _buzzer;
    ToneID _current_tone_id;
    uint8_t _tone_index;
    uint32_t _elapsed_tone_time;
    uint32_t _elapsed_half_period;
    uint32_t _half_period;
    uint32_t _last_update_time;
    volatile bool _tone_state;
    ToneFlags _flags;

    void play_tone(ToneID id);
    void update_flags();
    void update_next_note();

    static const Tone _tones[TONE_COUNT];
    static ToneAlarm *_tone_alarm_instance; // singleton pointer for ISR
    static void timer_task();               // ISR for IntervalTimer
};

#include "ToneAlarm.h"

// Tones string from Ardupilot are converted to {frquency, duration}
// and equation for note_period, _silence_duration and note_frequency from the Ardupilot parser

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

const ToneAlarm::Note none_notes[] = {{0, 0}};
const ToneAlarm::Note startup_notes[] = {{440, 109}, {0, 16}, {587, 109}, {0, 16}, {523, 109}, {0, 16}, {440, 109}, {0, 16}, {587, 109}, {0, 16}, {523, 109}, {0, 16}, {440, 109}, {0, 16}, {587, 109}, {0, 16}, {523, 109}, {0, 16}, {294, 55}, {0, 8}, {262, 55}, {0, 8}, {294, 55}, {0, 8}, {262, 55}, {0, 8}, {294, 55}, {0, 8}, {262, 55}, {0, 8}, {294, 55}, {0, 8}, {262, 55}, {0, 8}};
const ToneAlarm::Note arming_notes[] = {{392, 2800}, {0, 400}};
const ToneAlarm::Note disarming_notes[] = {{1045, 225}, {0, 75}};
const ToneAlarm::Note low_batt_notes[] = {{3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}, {3729, 103}, {0, 15}};
const ToneAlarm::Note failsafe_radio_notes[] = {{277, 225}, {0, 75}, {247, 450}, {0, 150}};

const ToneAlarm::Tone ToneAlarm::_tones[ToneAlarm::TONE_COUNT] = {
    {ToneAlarm::TONE_NONE, none_notes, ARRAY_SIZE(none_notes), false},
    {ToneAlarm::TONE_STARTUP, startup_notes, ARRAY_SIZE(startup_notes), false},
    {ToneAlarm::TONE_ARMING, arming_notes, ARRAY_SIZE(arming_notes), false},
    {ToneAlarm::TONE_DISARMING, disarming_notes, ARRAY_SIZE(disarming_notes), false},
    {ToneAlarm::TONE_LOW_BATT, low_batt_notes, ARRAY_SIZE(low_batt_notes), false},
    {ToneAlarm::TONE_FAILSAFE_RADIO, failsafe_radio_notes, ARRAY_SIZE(failsafe_radio_notes), false}};

ToneAlarm::ToneFlags ToneAlarm::events = {false, false, false, false};
ToneAlarm *ToneAlarm::_tone_alarm_instance = nullptr;

ToneAlarm::ToneAlarm(BuzzerDriver &buzzer)
{
    _buzzer = buzzer;
    _current_tone_id = TONE_NONE;
    _tone_index = 0;
    _elapsed_tone_time = 0;
    _elapsed_half_period = 0;
    _half_period = 0;
    _tone_state = false;
    _last_update_time = 0;
    _tone_alarm_instance = this;
}

void ToneAlarm::init()
{
    _buzzer.init();
    _toneTimer.begin(timer_task, 1000);
    play_tone(TONE_STARTUP);
}

void ToneAlarm::update()
{
    update_flags();
}

void ToneAlarm::play_tone(ToneID id)
{
    _current_tone_id = id;
    _tone_index = 0;
    _elapsed_tone_time = 0;

    uint16_t current_frequency = _tones[id].notes[_tone_index].frequency;
    if (current_frequency > 0)
    {
        _buzzer.start_tone(current_frequency);
    }
    else
    {
        _buzzer.stop_tone();
    }

    _last_update_time = micros();
}

void ToneAlarm::update_flags()
{
    if (_flags.armed != events.armed)
    {
        _flags.armed = events.armed;
        if (_flags.armed)
        {
            play_tone(TONE_ARMING);
        }
        else
        {
            play_tone(TONE_DISARMING);
        }
    }

    if (_flags.failsafe_radio != events.failsafe_radio)
    {
        _flags.failsafe_radio = events.failsafe_radio;
        if (_flags.failsafe_radio)
        {
            play_tone(TONE_FAILSAFE_RADIO);
        }
    }

    if (_flags.failsafe_battery != events.failsafe_battery)
    {
        _flags.failsafe_battery = events.failsafe_radio;
        if (_flags.failsafe_battery)
        {
            play_tone(TONE_LOW_BATT);
        }
    }
}

void ToneAlarm::update_next_note()
{
    uint32_t now = micros();
    uint32_t dt = now - _last_update_time;
    _last_update_time = now;

    if (_current_tone_id == TONE_NONE)
    {
        return;
    }

    const Note &note = _tones[_current_tone_id].notes[_tone_index];
    _elapsed_tone_time = _elapsed_tone_time + dt;

    if (_elapsed_tone_time >= note.duration * 1000UL)
    {
        _tone_index++;
        _elapsed_tone_time = 0;

        if (_tone_index >= _tones[_current_tone_id].length)
        {
            if (_tones[_current_tone_id].continuous)
            {
                _tone_index = 0;
            }
            else
            {
                stop_tone();
                return;
            }
        }

        const Note &next_note = _tones[_current_tone_id].notes[_tone_index];
        if (next_note.frequency > 0)
        {
            _buzzer.start_tone(next_note.frequency);
        }
        else
        {
            _buzzer.stop_tone();
        }
    }
}

void ToneAlarm::stop_tone()
{
    _current_tone_id = TONE_NONE;
    _buzzer.stop_tone();
}

void ToneAlarm::timer_task()
{
    if (_tone_alarm_instance)
    {
        _tone_alarm_instance->update_next_note();
    }
}

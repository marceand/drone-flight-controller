#include "ToneAlarm.h"

// Tones string from Ardupilot are converted to {frquency, duration}
// and equation for note_period, _silence_duration and note_frequency from the Ardupilot parser
// were used and _octave = 0

const ToneAlarm::Tone ToneAlarm::_tones[ToneAlarm::TONE_COUNT] = {
    {ToneAlarm::TONE_NONE, {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}, 5, false},
    {ToneAlarm::TONE_STARTUP, {{440, 109}, {587, 109}, {523, 109}, {440, 109}, {587, 109}, {523, 109}, {440, 109}, {587, 109}, {523, 109}, {587, 55}, {523, 55}, {587, 55}, {523, 55}, {587, 55}, {523, 55}, {587, 55}, {523, 55}, {587, 55}, {523, 55}, {587, 55}, {523, 55}}, 21, false},
    {ToneAlarm::TONE_ARMING, {{98, 2800}}, 1, false},
    {ToneAlarm::TONE_DISARMING, {{262, 262}}, 1, false},
    {ToneAlarm::TONE_LOW_BATT, {{466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}, {466, 103}}, 18, false},
    {ToneAlarm::TONE_FAILSAFE_RADIO, {{262, 262}, {117, 262}}, 2, false}};

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
    _last_update_time = micros();
}

void ToneAlarm::update()
{
    update_flags();
    update_next_note();
}

void ToneAlarm::play_tone(ToneID id)
{
    _current_tone_id = id;
    _tone_index = 0;
    _elapsed_tone_time = 0;

    uint16_t current_frequency = _tones[id].notes[_tone_index].frequency;
    _half_period = calculate_half_period(current_frequency);
    if (current_frequency > 0)
    {
        _toneTimer.begin(isrToggle, _half_period);
    }
    else
    {
        _toneTimer.end();
        _buzzer.disableTone();
    }

    _last_update_time = micros();
}

void ToneAlarm::update_flags()
{
    if (events.startup)
    {
        play_tone(TONE_STARTUP);
        events.startup = false;
    }

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
        _half_period = calculate_half_period(next_note.frequency);

        if (next_note.frequency > 0)
        {
            _toneTimer.begin(isrToggle, _half_period);
        }
        else
        {
            _toneTimer.end();
            _buzzer.disableTone();
        }
    }
}

void ToneAlarm::stop_tone()
{
    _current_tone_id = TONE_NONE;
    _toneTimer.end();
    _buzzer.disableTone();
}

void ToneAlarm::isrToggle()
{
    if (_tone_alarm_instance)
    {
        _tone_alarm_instance->_tone_state = !_tone_alarm_instance->_tone_state;
        if (_tone_alarm_instance->_tone_state)
        {
            _tone_alarm_instance->_buzzer.enableTone();
        }
        else
        {
            _tone_alarm_instance->_buzzer.disableTone();
        }
    }
}

uint32_t ToneAlarm::calculate_half_period(uint16_t frequency)
{
    if (frequency == 0)
    {
        return 0;
    }
    return 1000000UL / (2UL * frequency);
}

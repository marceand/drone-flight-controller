#include "ToneAlarm.h"

// const ToneAlarm::Tone ToneAlarm::_tones[ToneAlarm::TONE_COUNT] = {
//     {ToneAlarm::TONE_NONE, {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}, 5},
//     {ToneAlarm::TONE_STARTUP, {{440, 125}, {523, 125}, {659, 125}, {440, 125}, {0, 100}, {659, 250}, {784, 125}, {880, 125}, {784, 125}, {659, 125}, {523, 125}, {440, 125}, {0, 200}, {440, 500}, {0, 300}, {660, 400}, {880, 400}, {0, 300}, {659, 250}, {523, 250}, {440, 500}}, 21},
//     {ToneAlarm::TONE_ARMING, {{1000, 700}, {0, 100}, {1500, 200}, {0, 0}, {0, 0}}, 5},
//     {ToneAlarm::TONE_DISARMING, {{800, 200}, {0, 100}, {600, 200}, {0, 0}, {0, 0}}, 5},
//     {ToneAlarm::TONE_FAILSAFE, {{200, 300}, {0, 50}, {200, 300}, {0, 50}, {200, 300}}, 5}};

const ToneAlarm::Tone ToneAlarm::_tones[ToneAlarm::TONE_COUNT] = {
    {ToneAlarm::TONE_NONE, {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}, 5},
    {ToneAlarm::TONE_STARTUP, {{440, 125}, {587, 125}, {523, 125}, {440, 125}, {587, 125}, {523, 125}, {440, 125}, {587, 125}, {523, 125}, {587, 63}, {523, 63}, {587, 63}, {523, 63}, {587, 63}, {523, 63}, {587, 63}, {523, 63}}, 17},
    {ToneAlarm::TONE_ARMING, {{98, 3200}}, 1},
    {ToneAlarm::TONE_DISARMING, {{800, 200}, {0, 100}, {600, 200}, {0, 0}, {0, 0}}, 5},
    {ToneAlarm::TONE_FAILSAFE, {{200, 300}, {0, 50}, {200, 300}, {0, 50}, {200, 300}}, 5}};

// {440, 125}, {587, 125}, {523, 125},
// {440, 125}, {587, 125}, {523, 125},
// {440, 125}, {587, 125}, {523, 125},
// {587, 63}, {523, 63}, {587, 63}, {523, 63},
// {587, 63}, {523, 63}, {587, 63}, {523, 63}

//  {98, 3200}

// {932, 118}, {932, 118}, {932, 118}, {932, 118},
//     {932, 118}, {932, 118}, {932, 118}, {932, 118},
//     {932, 118}, {932, 118}, {932, 118}, {932, 118},
//     {932, 118}, {932, 118}, {932, 118}, {932, 118}

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
            _current_tone_id = TONE_NONE;
            _toneTimer.end();
            _buzzer.disableTone();
            return;
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

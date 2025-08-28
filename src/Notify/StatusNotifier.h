#pragma once

#include "ToneAlarm.h"
#include "LEDIndicator.h"

class StatusNotifier
{
public:
    StatusNotifier(ToneAlarm &toneAlarm,
                   LEDIndicator &ledIndicator) : _toneAlarm(toneAlarm),
                                                 _ledIndicator(ledIndicator) {}
    void init();
    void update();

    struct Events
    {
        bool armed = false;
        bool failsafe_radio = false;
        bool failsafe_battery = false;
        bool esc_calibration = false;
    };
    static Events events;

private:
    ToneAlarm &_toneAlarm;
    LEDIndicator &_ledIndicator;
};

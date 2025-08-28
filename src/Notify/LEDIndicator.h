#pragma once
#include <Arduino.h>
#include "../HAL/LEDDriver.h"

class LEDIndicator
{
public:
    LEDIndicator(LEDDriver &led) : _led(led) {}

    void init();
    void update(); // call at 50Hz

private:
    struct LEDFlags
    {
        bool armed = false;
        bool failsafe_radio = false;
        bool battery_low = false;
        bool esc_calibration = false;
    };

    enum class LedEvent : uint8_t
    {
        NONE,
        STARTUP,
        ARMING,
        DISARMING,
        ARMED,
        DISARMED,
        ESC_CALIBRATION,
        FAILSAFE_RADIO,
        LOW_BATTERY
    };

    struct LedPattern
    {
        uint16_t onMs;
        uint16_t offMs;
        uint16_t durationMs; // how long this pattern runs (0 = infinite)
        bool repeat;
        bool is_red_activated;
    };

    void setEvent(LedEvent ev);
    void applyPattern(const LedPattern &pat);
    void transition_to_steady_state();
    LedPattern patternForEvent(LedEvent ev);

    uint8_t _redPin, _greenPin;
    LedEvent _currentEvent;
    LedEvent _activeEvent;

    // timing state
    uint32_t _lastToggle;
    uint32_t _startTime;
    bool _ledState;
    LedPattern _currentPattern;
    LEDFlags _flags;
    LEDDriver &_led;
};

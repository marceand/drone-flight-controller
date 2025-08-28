#include "LEDIndicator.h"
#include "StatusNotifier.h"

// ---------- Begin ----------
void LEDIndicator::init()
{
    _led.init();
    setEvent(LedEvent::NONE);
}

void LEDIndicator::update()
{
    uint32_t now = millis();

    // === Detect state changes ===
    if (_flags.armed != StatusNotifier::events.armed)
    {
        _flags.armed = StatusNotifier::events.armed;
        setEvent(_flags.armed ? LedEvent::ARMING : LedEvent::DISARMING);
    }

    if (_flags.failsafe_radio != StatusNotifier::events.failsafe_radio)
    {
        _flags.failsafe_radio = StatusNotifier::events.failsafe_radio;
        if (_flags.failsafe_radio)
        {
            setEvent(LedEvent::FAILSAFE_RADIO);
        }
        else
        {
            setEvent(_flags.armed ? LedEvent::ARMED : LedEvent::DISARMED);
        }
    }

    if (_flags.battery_low != StatusNotifier::events.failsafe_battery)
    {
        _flags.battery_low = StatusNotifier::events.failsafe_battery;
        if (_flags.battery_low)
        {
            setEvent(LedEvent::LOW_BATTERY);
        }
        else
        {
            setEvent(_flags.armed ? LedEvent::ARMED : LedEvent::DISARMED);
        }
    }

    if (_flags.esc_calibration != StatusNotifier::events.esc_calibration)
    {
        _flags.esc_calibration = StatusNotifier::events.esc_calibration;
        if (_flags.esc_calibration)
        {
            setEvent(LedEvent::ESC_CALIBRATION);
        }
        else
        {
            setEvent(_flags.armed ? LedEvent::ARMED : LedEvent::DISARMED);
        }
    }

    // === Handle LED pattern ===
    if (_activeEvent == LedEvent::NONE)
        return;

    if (_currentPattern.durationMs > 0 &&
        now - _startTime > _currentPattern.durationMs)
    {
        transition_to_steady_state();
        return;
    }

    uint16_t interval = _ledState ? _currentPattern.onMs : _currentPattern.offMs;
    if (interval == 0)
    {

        if (_currentPattern.is_red_activated)
        {
            _led.enableRedLED();
            _led.disableGreenLED();
        }
        else
        {
            _led.enableGreenLED();
            _led.disableRedLED();
        }
        return;
    }

    if (now - _lastToggle >= interval)
    {
        _ledState = !_ledState;
        _lastToggle = now;

        if (_ledState)
        {
            if (_currentPattern.is_red_activated)
            {
                _led.enableRedLED();
                _led.disableGreenLED();
            }
            else
            {
                _led.enableGreenLED();
                _led.disableRedLED();
            }
        }
        else
        {
            _led.disableRedLED();
            _led.disableGreenLED();
        }
    }
}

void LEDIndicator::setEvent(LedEvent ev)
{
    if (ev != _activeEvent)
    {
        _currentEvent = ev;
        applyPattern(patternForEvent(ev));
    }
}

// ---------- Pattern Applier ----------
void LEDIndicator::applyPattern(const LedPattern &pat)
{
    _currentPattern = pat;
    _startTime = millis();
    _lastToggle = millis();
    _ledState = false;
    _activeEvent = _currentEvent;
}

// ---------- Fallback ----------
void LEDIndicator::transition_to_steady_state()
{
    if (_activeEvent == LedEvent::ARMING)
    {
        setEvent(LedEvent::ARMED);
    }
    else if (_activeEvent == LedEvent::DISARMING)
    {
        setEvent(LedEvent::DISARMED);
    }
}

// ---------- Pattern Lookup ----------
LEDIndicator::LedPattern LEDIndicator::patternForEvent(LedEvent ev)
{
    switch (ev)
    {
    case LedEvent::STARTUP:
        return {100, 100, 600, true, true}; // red blink x3
    case LedEvent::ARMING:
        return {200, 200, 2000, true, false}; // green blink
    case LedEvent::DISARMING:
        return {200, 800, 2000, true, false}; // slow green blink
    case LedEvent::ARMED:
        return {1000, 0, 0, false, false}; // solid green
    case LedEvent::DISARMED:
        return {200, 1800, 0, true, false}; // green blink every 2s
    case LedEvent::ESC_CALIBRATION:
        return {200, 200, 0, true, true}; // fast red blink
    case LedEvent::FAILSAFE_RADIO:
        return {100, 400, 0, true, true}; // red blink with pause
    case LedEvent::LOW_BATTERY:
        return {100, 100, 0, true, true}; // very fast red blink
    default:
        return {0, 0, 0, false, false};
    }
}

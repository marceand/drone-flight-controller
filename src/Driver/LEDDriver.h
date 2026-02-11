#pragma once

class LEDDriver
{
public:
    void init();
    void enableRedLED();
    void disableRedLED();
    void enableGreenLED();
    void disableGreenLED();

private:
    bool is_initialized = false;
};

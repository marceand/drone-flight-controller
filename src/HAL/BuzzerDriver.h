#pragma once

class BuzzerDriver
{
public:
    void init();
    void enableTone();
    void disableTone();

private:
    bool is_initialized = false;
};

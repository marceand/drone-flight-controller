#pragma once

#include <EEPROM.h>

#define CONFIG_MAGIC_NUMBER_BYTE 0xAA

class EepromStorage
{
public:
    void init();
    bool check_for_esc_calibration();
    void set_check_esc_calibration(bool check_for_calibration);
    uint32_t get_session_id();

private:
    struct Config
    {
        uint8_t initialized;
        bool check_esc_calibration;
        uint32_t boot_count;
    };

    Config _config;

    void setDefaults();
    void load();
    void save();
    void update_boot_count();
};

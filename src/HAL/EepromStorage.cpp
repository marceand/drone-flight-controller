#include <Arduino.h>
#include "EepromStorage.h"

void EepromStorage::init()
{
    load();
    if (_config.initialized != CONFIG_MAGIC_NUMBER_BYTE)
    {
        setDefaults();
        save();
    }

    // Serial.print("before boot count in eeprom: ");
    // Serial.println(_config.boot_count);
    update_boot_count();
    // Serial.print("after boot count in eeprom: ");
    // Serial.println(_config.boot_count);
}

bool EepromStorage::check_for_esc_calibration()
{
    return _config.check_esc_calibration;
}

void EepromStorage::set_check_esc_calibration(bool check_for_calibration)
{
    if (_config.check_esc_calibration != check_for_calibration)
    {
        _config.check_esc_calibration = check_for_calibration;
        save();
    }
}

uint32_t EepromStorage::get_session_id()
{
    return _config.boot_count;
}

void EepromStorage::setDefaults()
{
    _config.initialized = CONFIG_MAGIC_NUMBER_BYTE;
    _config.check_esc_calibration = true;
    _config.boot_count = 0;
}

void EepromStorage::load()
{
    EEPROM.get(0, _config);
}

void EepromStorage::save()
{
    EEPROM.put(0, _config);
}

void EepromStorage::update_boot_count()
{
    _config.boot_count++;
    save();
}

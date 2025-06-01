#include "EepromStorage.h"

void EepromStorage::init()
{
    load();
    if (_config.initialized != CONFIG_MAGIC_NUMBER_BYTE)
    {
        setDefaults();
        save();
    }
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

void EepromStorage::setDefaults()
{
    _config.initialized = CONFIG_MAGIC_NUMBER_BYTE;
    _config.check_esc_calibration = true;
}

void EepromStorage::load()
{
    EEPROM.get(0, _config);
}

void EepromStorage::save()
{
    EEPROM.put(0, _config);
}

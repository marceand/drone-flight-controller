#include "PersistentStorage.h"

void PersistentStorage::init()
{
    load();
    if (_config.initialized != CONFIG_MAGIC_NUMBER_BYTE)
    {
        setDefaults();
        save();
    }
}

bool PersistentStorage::check_for_esc_calibration()
{
    return _config.check_esc_calibration;
}

void PersistentStorage::set_check_esc_calibration(bool check_for_calibration)
{
    if (_config.check_esc_calibration != check_for_calibration)
    {
        _config.check_esc_calibration = check_for_calibration;
        save();
    }
}

void PersistentStorage::setDefaults()
{
    _config.initialized = CONFIG_MAGIC_NUMBER_BYTE;
    _config.check_esc_calibration = true;
}

void PersistentStorage::load()
{
    EEPROM.get(0, _config);
}

void PersistentStorage::save()
{
    EEPROM.put(0, _config);
}

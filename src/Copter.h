#pragma once

#include "RC_Channels/RC_Channels.h"
#include "InertialSensor/InertialSensor.h"
#include "KalmanFilter/RollPitchAngleKF.h"
#include "Controller/RateController.h"
#include "Controller/AngleController.h"
#include "BatteryMonitor/BatteryMonitor.h"
#include "Motors/Motors.h"
#include "HAL/PersistentStorage.h"
#include "HAL/LEDIndicator.h"
#include "Barometer/Barometer_BMP280.h"
#include "KalmanFilter/AltitudeVelocityKF.h"
#include "PID/CopterPID.h"

class Copter
{
public:
    Copter(RC_Channels &rc,
           InertialSensor &inertialSensor,
           RateController &rateController,
           AngleController &angleController,
           RollPitchAngleKF &rollPitchAngleKF,
           BatteryMonitor &battMonitor,
           Motors &motors,
           PersistentStorage &storage,
           LEDIndicator &led,
           Barometer_BMP280 &barometer,
           AltitudeVelocityKF &altitudeVelocityKF,
           CopterPID &velocityController) : _rc(rc),
                                            _inertialSensor(inertialSensor),
                                            _rateController(rateController),
                                            _angleController(angleController),
                                            _rollPitchAngleKF(rollPitchAngleKF),
                                            _battMonitor(battMonitor),
                                            _motors(motors),
                                            _storage(storage),
                                            _ledIndicator(led),
                                            _barometer(barometer),
                                            _altitudeVelocityKF(altitudeVelocityKF),
                                            _velocityController(velocityController)

    {
    }

    void init(void);
    void run(void);

private:
    RC_Channels &_rc;
    InertialSensor &_inertialSensor;
    RateController &_rateController;
    AngleController &_angleController;
    RollPitchAngleKF &_rollPitchAngleKF;
    BatteryMonitor &_battMonitor;
    Motors &_motors;
    PersistentStorage &_storage;
    LEDIndicator &_ledIndicator;
    Barometer_BMP280 &_barometer;
    AltitudeVelocityKF &_altitudeVelocityKF;
    CopterPID &_velocityController;

    void check_esc_calibration();
};
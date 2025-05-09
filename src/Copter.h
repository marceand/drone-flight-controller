#pragma once

#include "RC_Channels/RC_Channels.h"
#include "InertialSensor/InertialSensor.h"
#include "KalmanFilter/RollPitchAngleKF.h"
#include "Controller/RateController.h"
#include "Controller/AngleController.h"
#include "BatteryMonitor/BatteryMonitor.h"

class Copter
{
public:
    Copter(RC_Channels &rc,
           InertialSensor &inertialSensor,
           RateController &rateController,
           AngleController &angleController,
           RollPitchAngleKF &rollPitchAngleKF,
           BatteryMonitor &battMonitor) : _rc(rc),
                                          _inertialSensor(inertialSensor),
                                          _rateController(rateController),
                                          _angleController(angleController),
                                          _rollPitchAngleKF(rollPitchAngleKF),
                                          _battMonitor(battMonitor) {}

    void init(void);
    void run(void);

private:
    RC_Channels &_rc;
    InertialSensor &_inertialSensor;
    RateController &_rateController;
    AngleController &_angleController;
    RollPitchAngleKF &_rollPitchAngleKF;
    BatteryMonitor &_battMonitor;
    void check_esc_calibration();
};
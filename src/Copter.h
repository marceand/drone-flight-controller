#pragma once

#include "RC_Channels/RC_Channels.h"
#include "InertialSensor/InertialSensor.h"
#include "BatteryMonitor/BatteryMonitor.h"
#include "Motors/Motors.h"
#include "HAL/EepromStorage.h"
#include "HAL/LEDIndicator.h"
#include "Barometer/Barometer_BMP280.h"
#include "KalmanFilter/AltitudeVelocityKF.h"
#include "PID/CopterPID.h"
#include "KalmanFilter/AngleKF.h"
#include "Functor.h"

class Copter
{
public:
    Copter(RC_Channels &rc,
           InertialSensor &inertialSensor,
           CopterPID &rateRollController,
           CopterPID &ratePitchController,
           CopterPID &rateYawController,
           CopterPID &angleRollController,
           CopterPID &anglePitchController,
           BatteryMonitor &battMonitor,
           Motors &motors,
           EepromStorage &storage,
           LEDIndicator &led,
           Barometer_BMP280 &barometer,
           AltitudeVelocityKF &altitudeVelocityKF,
           CopterPID &velocityController,
           AngleKF &rollKF,
           AngleKF &pitchKF) : _rc(rc),
                               _inertialSensor(inertialSensor),
                               _rateRollController(rateRollController),
                               _ratePitchController(ratePitchController),
                               _rateYawController(rateYawController),
                               _angleRollController(angleRollController),
                               _anglePitchController(anglePitchController),
                               _battMonitor(battMonitor),
                               _motors(motors),
                               _storage(storage),
                               _ledIndicator(led),
                               _barometer(barometer),
                               _altitudeVelocityKF(altitudeVelocityKF),
                               _velocityController(velocityController),
                               _rollKF(rollKF),
                               _pitchKF(pitchKF)

    {
    }

    struct Task
    {
        const char *name;
        uint32_t interval_us;
        uint32_t last_run_us;
        Functor<Copter> function;
    };

    static Task tasks[];
    const static int NUM_TASKS;
    void init(void);
    void run(void);

private:
    RC_Channels &_rc;
    InertialSensor &_inertialSensor;
    CopterPID &_rateRollController;
    CopterPID &_ratePitchController;
    CopterPID &_rateYawController;
    CopterPID &_angleRollController;
    CopterPID &_anglePitchController;
    BatteryMonitor &_battMonitor;
    Motors &_motors;
    EepromStorage &_storage;
    LEDIndicator &_ledIndicator;
    Barometer_BMP280 &_barometer;
    AltitudeVelocityKF &_altitudeVelocityKF;
    CopterPID &_velocityController;
    AngleKF &_rollKF;
    AngleKF &_pitchKF;

    int _arming_counter = 0;
    bool is_flying = false;
    float _input[4] = {0.0f};

    void read_rc_channels();
    void read_inertial_sensor();
    void read_barometer();
    void check_takeoff();
    void run_main_controller();
    void run_motors();
    // void run_battery_monitor();
    void check_esc_calibration();
    void check_motors_startup();
    void check_motors_mapping();
    void arm_esc_at_minimum();
    void check_motors_arming();
    // void check_disarming();
};
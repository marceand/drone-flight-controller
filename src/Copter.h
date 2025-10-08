#pragma once

#include "RC_Channels/RC_Channels.h"
#include "InertialSensor/InertialSensor.h"
#include "BatteryMonitor/BatteryMonitor.h"
#include "Motors/Motors.h"
#include "HAL/EepromStorage.h"
#include "Barometer/Barometer_BMP280.h"
#include "Attitude/AttitudeEstimator.h"
#include "Attitude/AttitudeController.h"
#include "Vertical/VerticalEstimator.h"
#include "Vertical/VerticalVelocityController.h"
#include "Functor.h"
#include "Notify/StatusNotifier.h"

class Copter
{
public:
    Copter(RC_Channels &rc,
           InertialSensor &inertialSensor,
           BatteryMonitor &battMonitor,
           Motors &motors,
           EepromStorage &storage,
           Barometer_BMP280 &barometer,
           AttitudeEstimator &attitudeEstimator,
           AttitudeController &attitudeController,
           VerticalEstimator &verticalEstimator,
           VerticalVelocityController &verticalVelocityController,
           StatusNotifier &notifier) : _rc(rc),
                                       _inertialSensor(inertialSensor),
                                       _battMonitor(battMonitor),
                                       _motors(motors),
                                       _storage(storage),
                                       _barometer(barometer),
                                       _attitudeEstimator(attitudeEstimator),
                                       _attitudeController(attitudeController),
                                       _verticalEstimator(verticalEstimator),
                                       _verticalVelocityController(verticalVelocityController),
                                       _notifier(notifier)

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

private:
    RC_Channels &_rc;
    InertialSensor &_inertialSensor;
    BatteryMonitor &_battMonitor;
    Motors &_motors;
    EepromStorage &_storage;
    Barometer_BMP280 &_barometer;
    AttitudeEstimator &_attitudeEstimator;
    AttitudeController &_attitudeController;
    VerticalEstimator &_verticalEstimator;
    VerticalVelocityController &_verticalVelocityController;
    StatusNotifier &_notifier;

    int _arming_counter = 0;
    bool is_flying = false;
    float _input[4] = {0.0f};
    uint32_t notifier_update_ms = 0;

    void read_rc_channels();
    void read_inertial_sensor();
    void read_barometer();
    void check_takeoff();
    void run_main_controller();
    void run_motors();
    void run_battery_monitor();
    void check_esc_calibration();
    void check_motors_startup();
    void check_motors_mapping();
    void arm_esc_at_minimum();
    void check_motors_arming();
    bool is_pre_arm_check_pass();
    void run_notifier();
    // void check_disarming();
    void update_notifier();
    void update_logging();
};
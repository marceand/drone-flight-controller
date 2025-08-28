#include <Wire.h>
#include "src/Copter.h"
#include "src/RC_Channels/RC_Channels.h"
#include "src/InertialSensor/InertialSensor.h"
#include "src/Parameters/Parameters.h"
#include "src/BatteryMonitor/BatteryMonitor.h"
#include "src/Motors/Motors.h"
#include "src/HAL/ESCOutput.h"
#include "src/HAL/EepromStorage.h"
#include "src/Barometer/Barometer_BMP280.h"
#include "src/KalmanFilter/AltitudeVelocityKF.h"
#include "src/KalmanFilter/AngleKF.h"
#include "src/PID/CopterPID.h"
#include "src/Attitude/AttitudeEstimator.h"
#include "src/Attitude/AttitudeController.h"
#include "src/Vertical/VerticalEstimator.h"
#include "src/Vertical/VerticalVelocityController.h"
#include "src/HAL/BuzzerDriver.h"
#include "src/HAL/LEDDriver.h"
#include "src/Notify/ToneAlarm.h"
#include "src/Notify/LEDIndicator.h"
#include "src/Notify/StatusNotifier.h"

#define WIRE_CLK_FREQ 400000 // 400Khz
#define SERIAL_BAUD_RATE 57600
#define HZ_TO_US(hz) (1000000UL / (hz))

RC_Channels rc(&Serial1);
InertialSensor inertialSensor;
CopterPID rateRollPID;
CopterPID ratePitchPID;
CopterPID rateYawPID;
CopterPID angleRollPID;
CopterPID anglePitchPID;
BatteryMonitor battMonitor;
ESCOutput escOutput;
Motors motors(escOutput);
EepromStorage storage;
Barometer_BMP280 barometer;
AltitudeVelocityKF altitudeVelocityKF;
CopterPID velocityPID;
AngleKF rollKF;
AngleKF pitchKF;
AttitudeEstimator attitudeEstimator(inertialSensor, rollKF, pitchKF);
AttitudeController attitudeController(rateRollPID, ratePitchPID, rateYawPID, angleRollPID, anglePitchPID);
VerticalEstimator verticalEstimator(barometer, altitudeVelocityKF, inertialSensor, attitudeEstimator);
VerticalVelocityController verticalVelocityController(velocityPID);
BuzzerDriver buzzer;
ToneAlarm toneAlarm(buzzer);
LEDDriver led;
LEDIndicator ledIndicator(led);
StatusNotifier notifier(toneAlarm, ledIndicator);

Copter copter(
    rc,
    inertialSensor,
    battMonitor,
    motors,
    storage,
    barometer,
    attitudeEstimator,
    attitudeController,
    verticalEstimator,
    verticalVelocityController,
    notifier);

Copter::Task Copter::tasks[] = {
    // {"read_rc", HZ_TO_US(250), 0, Functor<Copter>(&copter, &Copter::read_rc_channels)},
    // {"read_inertial", HZ_TO_US(250), 0, Functor<Copter>(&copter, &Copter::read_inertial_sensor)},
    // {"read_barometer", HZ_TO_US(250), 0, Functor<Copter>(&copter, &Copter::read_barometer)},
    // {"check_takeoff", HZ_TO_US(250), 0, Functor<Copter>(&copter, &Copter::check_takeoff)},
    // {"run_main_controller", HZ_TO_US(250), 0, Functor<Copter>(&copter, &Copter::run_main_controller)},
    // {"run_motors", HZ_TO_US(250), 0, Functor<Copter>(&copter, &Copter::run_motors)},
    // {"check_motors_arming", HZ_TO_US(10), 0, Functor<Copter>(&copter, &Copter::check_motors_arming)},
    {"run_tone_alarm", HZ_TO_US(50), 0, Functor<Copter>(&copter, &Copter::run_notifier)},
};

const int Copter::NUM_TASKS = sizeof(tasks) / sizeof(Task);

unsigned long loopTimer = micros();

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    Wire.setClock(WIRE_CLK_FREQ);
    Wire.begin();
    delay(250);

    copter.init();

    // rcCheck();
}

void loop()
{
    uint32_t now = micros();

    for (int i = 0; i < Copter::NUM_TASKS; i++)
    {
        Copter::Task &task = Copter::tasks[i];
        if (now - task.last_run_us >= task.interval_us && task.function.valid())
        {
            task.function();
            task.last_run_us = now;
        }
    }
}

#include <Wire.h>
#include "src/Copter.h"
#include "src/RC_Channels/RC_Channels.h"
#include "src/InertialSensor/InertialSensor.h"
#include "src/Parameters/Parameters.h"
#include "src/BatteryMonitor/BatteryMonitor.h"
#include "src/Motors/Motors.h"
#include "src/HAL/ESCOutput.h"
#include "src/HAL/EepromStorage.h"
#include "src/HAL/LEDIndicator.h"
#include "src/Barometer/Barometer_BMP280.h"
#include "src/KalmanFilter/AltitudeVelocityKF.h"
#include "src/KalmanFilter/AngleKF.h"
#include "src/PID/CopterPID.h"

#define WIRE_CLK_FREQ 400000 // 400Khz
#define SERIAL_BAUD_RATE 57600

#define LOOP_250_HZ 4000  // run control loop every 4 ms (250 HZ)
#define LOOP_10_HZ 100000 // run control loop every 4 ms (250 HZ)
#define HZ_TO_US(hz) (1000000UL / (hz))

// Copter copter; // your flight controller instance
// Task tasks[] = {
//     {"CopterRun", 10000, 0, Functor<Copter>(&copter, &Copter::run)},
//     // Add more tasks with other methods if needed
// };

// void run_task_if_ready(Task &t, uint32_t now_us)
// {
//     if ((now_us - t.last_run_us) >= t.interval_us && t.func.valid())
//     {
//         t.last_run_us = now_us;
//         t.func(); // calls copter.run()
//     }
// }

RC_Channels rc(&Serial1);
InertialSensor inertialSensor;
CopterPID rateRollController;
CopterPID ratePitchController;
CopterPID rateYawController;
CopterPID angleRollController;
CopterPID anglePitchController;
LEDIndicator led;
BatteryMonitor battMonitor(led);
ESCOutput escOutput;
Motors motors(escOutput);
EepromStorage storage;
Barometer_BMP280 barometer;
AltitudeVelocityKF altitudeVelocityKF;
CopterPID velocityController;
AngleKF rollKF;
AngleKF pitchKF;

Copter copter(
    rc,
    inertialSensor,
    rateRollController,
    ratePitchController,
    rateYawController,
    angleRollController,
    anglePitchController,
    battMonitor,
    motors,
    storage,
    led,
    barometer,
    altitudeVelocityKF,
    velocityController,
    rollKF,
    pitchKF);

Copter::Task Copter::tasks[] = {
    {"TaskA", HZ_TO_US(10), 0, Functor<Copter>(&copter, &Copter::run)},
    // {"TaskA", HZ_TO_US(100), 0, taskA},
    // {"TaskB", HZ_TO_US(50), 0, taskB},
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

    // copter.init();
    battMonitor.init();

    // rcCheck();
}

void loop()
{
    // copter.run();
    battMonitor.monitor();

    Serial.print("Init-capacity: ");
    Serial.print(battMonitor.initial_capacity());
    Serial.print("unit \t");
    Serial.print("Voltage: ");
    Serial.print(battMonitor.voltage());
    Serial.print("V \t");
    Serial.print("Current: ");
    Serial.print(battMonitor.current(), 6);
    Serial.print("Amp \t");
    Serial.print("Percentage: ");
    Serial.print(battMonitor.get_remaining_percentage());
    Serial.println("%");

    while (micros() - loopTimer < LOOP_250_HZ)
        ;
    loopTimer = micros();

    // uint32_t now = micros();

    // for (int i = 0; i < Copter::NUM_TASKS; i++)
    // {
    //     Copter::Task &task = Copter::tasks[i];
    //     if (now - task.last_run_us >= task.interval_us && task.function.valid())
    //     {
    //         task.function();
    //         task.last_run_us = now;
    //     }
    // }

    // inertialSensor.read();
    // float rollRate = inertialSensor.getCalibGyroX();
    // float pitchRate = inertialSensor.getCalibGyroY();
    // float yawRate = inertialSensor.getCalibGyroZ();
    // float rollAngle = inertialSensor.getRollAngle();
    // float pitchAngle = inertialSensor.getPitchAngle();

    // float rollAngleKF = rollKF.calculateAngle(rollRate, rollAngle);
    // float pitchAngleKF = pitchKF.calculateAngle(pitchRate, pitchAngle);

    // Serial.print(rollKF.getGain(), 6);
    // Serial.print("cm \t");
    // Serial.print(pitchKF.getGain(), 6);
    // Serial.println();

    // inertialSensor.read();
    // barometer.read();
    // float vertical_acceleration = inertialSensor.getVerticalAcceleration();
    // float relative_altitude = barometer.get_relative_altitude_in_cm();

    // altitudeVelocityKF.calculateVerticalVelocity(relative_altitude, vertical_acceleration);

    // Serial.print("relative_altitude: ");
    // Serial.print(relative_altitude);
    // Serial.print("cm \t");
    // Serial.print("Vertical velocity: ");
    // vel = vel + vertical_acceleration * 0.004; // convert to cm/s
    // Serial.print(vel);
    // Serial.print("cm/s \t");
    // Serial.print("Altitude: ");
    // Serial.print(altitudeVelocityKF.getAltitude());
    // Serial.print("cm \t");
    // Serial.print("Velocity: ");
    // Serial.print(altitudeVelocityKF.getVerticalVelocity());
    // Serial.println("cm/s");

    // delay(20);
    // copter.run();
    // while (micros() - loopTimer < LOOP_10_HZ)
    //     ;
    // loopTimer = micros();

    // barometer.read();
    // float reference_altitude = barometer.get_reference_altitude_in_cm();
    // float pressure_altitude = barometer.get_pressure_altitude_in_cm();
    // float relative_altitude = barometer.get_relative_altitude_in_cm();
    // Serial.print("Reference: ");
    // Serial.print(reference_altitude);
    // Serial.print("cm \t");
    // Serial.print("Pressure: ");
    // Serial.print(pressure_altitude);
    // Serial.print("cm \t");
    // Serial.print("Relative: ");
    // Serial.print(relative_altitude);
    // Serial.println("cm");
    // delay(1000);
    // copter.run();
    // while (micros() - loopTimer < LOOP_250_HZ)
    //     ;
    // loopTimer = micros();
}

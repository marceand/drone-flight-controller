#include <Wire.h>
#include "src/Copter.h"
#include "src/RC_Channels/RC_Channels.h"
#include "src/InertialSensor/InertialSensor.h"
#include "src/KalmanFilter/RollPitchAngleKF.h"
#include "src/Controller/RateController.h"
#include "src/Controller/AngleController.h"
#include "src/Parameters/Parameters.h"
#include "src/BatteryMonitor/BatteryMonitor.h"
#include "src/Motors/Motors.h"
#include "src/HAL/ESCOutput.h"
#include "src/HAL/PersistentStorage.h"
#include "src/HAL/LEDIndicator.h"
#include "src/Barometer/Barometer_BMP280.h"
#include "src/KalmanFilter/AltitudeVelocityKF.h"

#define WIRE_CLK_FREQ 400000 // 400Khz
#define SERIAL_BAUD_RATE 57600

#define LOOP_250_HZ 4000 // run control loop every 4 ms (250 HZ)

RC_Channels rc(&Serial1);
InertialSensor inertialSensor;
RateController rateController;
AngleController angleController;
RollPitchAngleKF rollPitchAngleKF;
LEDIndicator led;
BatteryMonitor battMonitor(led);
ESCOutput escOutput;
Motors motors(escOutput);
PersistentStorage storage;
Barometer_BMP280 barometer;
AltitudeVelocityKF altitudeVelocityKF;

Copter copter(
    rc,
    inertialSensor,
    rateController,
    angleController,
    rollPitchAngleKF,
    battMonitor,
    motors,
    storage,
    led);

unsigned long loopTimer = micros();
float vel = 0.0f;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    Wire.setClock(WIRE_CLK_FREQ);
    Wire.begin();
    delay(250);

    inertialSensor.init();
    barometer.init();
    altitudeVelocityKF.setParameters();

    // copter.init();

    // rcCheck();
}

void loop()
{
    inertialSensor.read();
    barometer.read();
    float vertical_acceleration = inertialSensor.getVerticalAcceleration();
    float relative_altitude = barometer.get_relative_altitude_in_cm();

    altitudeVelocityKF.calculate_altitude_velocity(relative_altitude, vertical_acceleration);

    // Serial.print(altitudeVelocityKF.getAltitude());
    // Serial.print("\t");
    // Serial.print(altitudeVelocityKF.getVerticalVelocity());
    // Serial.println("\t");

    // Serial.print("Gain-Alt: ");
    // Serial.print(altitudeVelocityKF.getGainAltitude(), 6);
    // Serial.print("\t");
    // Serial.print("Gain-Vel: ");
    // Serial.print(altitudeVelocityKF.getGainVelocity(), 6);
    // Serial.println("\t");

    Serial.print("relative_altitude: ");
    Serial.print(relative_altitude);
    Serial.print("cm \t");
    Serial.print("Vertical velocity: ");
    vel = vel + vertical_acceleration * 0.004; // convert to cm/s
    Serial.print(vel);
    Serial.print("cm/s \t");
    Serial.print("Altitude: ");
    Serial.print(altitudeVelocityKF.getAltitude());
    Serial.print("cm \t");
    Serial.print("Velocity: ");
    Serial.print(altitudeVelocityKF.getVerticalVelocity());
    Serial.println("cm/s");

    // delay(20);
    while (micros() - loopTimer < LOOP_250_HZ)
        ;
    loopTimer = micros();

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

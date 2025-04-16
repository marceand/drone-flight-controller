#include "sbus.h"
#include <Wire.h>

#include "src/RC_Channels/RC_Channels.h"
#include "src/InertialSensor/InertialSensor.h"
#include "src/KalmanFilter/RollPitchAngleKF.h"
#include "src/Controller/RateController.h"
#include "src/Controller/AngleController.h"
#include "src/Copter.h"

#define MOTOR_STOP 1000
#define MOTOR_MAX_SPEED 1999

#define WIRE_CLK_FREQ 400000 // 400Khz
#define SERIAL_BAUD_RATE 57600

#define LOOP_250_HZ 4000 // run control loop every 4 ms (250 HZ)

typedef struct
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motors_t;

RC_Channels rc(&Serial2);
InertialSensor inertialSensor;
RateController rateController;
AngleController angleController;
RollPitchAngleKF rollPitchAngleKF;

Copter copter(rc,
                           inertialSensor,
                           rateController,
                           angleController,
                           rollPitchAngleKF);

unsigned long loopTimer = micros();

void runPid()
{
    // rc.read();
    // float desiredRollAngle = rc.getDesiredRollAngle();
    // float desiredPitchAngle = rc.getDesiredPitchAngle();
    // float desiredYawRate = rc.getDesiredYawRate();
    // float throttleInPWM = rc.getThrottleInPWM();

    // Serial.print("Roll:");
    // Serial.print(rc.getRollInPWM());
    // Serial.print("\t");

    // Serial.print("Pitch:");
    // Serial.print(rc.getPitchInPWM());
    // Serial.print("\t");

    // Serial.print("Yaw:");
    // Serial.print(rc.getYawInPWM());
    // Serial.print("\t");

    // Serial.print("Throttle:");
    // Serial.println(rc.getThrottleInPWM());

    // inertialSensor.read();

    

    // float rollInput = pid.computeRollPID(desiredRollRate, inertialSensor.getCalibGyroX());
    // float pitchInput = pid.computePitchPID(desiredPitchRate, inertialSensor.getCalibGyroY());
    // float yawInput = pid.computeYawPID(desiredYawRate, inertialSensor.getCalibGyroZ());

    // runMotors(throttleInPWM, rollInput, pitchInput, yawInput);
}

void runMotors(uint16_t throttle, float roll, float pitch, float yaw)
{
    motors_t motors;
    if (throttle > RC_MIN_THROTTLE)
    {
        motors.m1 = 1.024 * (throttle - roll - pitch - yaw);
        motors.m1 = motors.m1 > MOTOR_MAX_SPEED ? MOTOR_MAX_SPEED : motors.m1;
        motors.m2 = 1.024 * (throttle - roll + pitch + yaw);
        motors.m2 = motors.m2 > MOTOR_MAX_SPEED ? MOTOR_MAX_SPEED : motors.m2;
        motors.m3 = 1.024 * (throttle + roll + pitch - yaw);
        motors.m3 = motors.m3 > MOTOR_MAX_SPEED ? MOTOR_MAX_SPEED : motors.m3;
        motors.m4 = 1.024 * (throttle + roll - pitch + yaw);
        motors.m4 = motors.m4 > MOTOR_MAX_SPEED ? MOTOR_MAX_SPEED : motors.m4;
    }
    else
    {
        motors.m1 = MOTOR_STOP;
        motors.m2 = MOTOR_STOP;
        motors.m3 = MOTOR_STOP;
        motors.m4 = MOTOR_STOP;
    }

    updateMotorSpeeds(motors);
}

void updateMotorSpeeds(motors_t motors)
{
    // Serial.print("M1:");
    // Serial.print(motors.m1);
    // Serial.print("\t");

    // Serial.print("M2:");
    // Serial.print(motors.m2);
    // Serial.print("\t");

    // Serial.print("M3:");
    // Serial.print(motors.m3);
    // Serial.print("\t");

    // Serial.print("M4:");
    // Serial.println(motors.m4);
    // analogWrite(MOTOR_1_PIN, motors.m1);
    // analogWrite(MOTOR_2_PIN, motors.m2);
    // analogWrite(MOTOR_3_PIN, motors.m3);
    // analogWrite(MOTOR_4_PIN, motors.m4);
}

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
    copter.run();
    while (micros() - loopTimer < LOOP_250_HZ)
        ;
    loopTimer = micros();
}

#pragma once

#define MOTOR_1_PIN 1
#define MOTOR_2_PIN 2
#define MOTOR_3_PIN 3
#define MOTOR_4_PIN 4
#define SCALE_TO_12_BIT 1.024f

class ESCOutput
{
public:
    void init();
    void update_motor_1_speed(float input);
    void update_motor_2_speed(float input);
    void update_motor_3_speed(float input);
    void update_motor_4_speed(float input);

private:
    bool is_initalized = false;
};

#include "MotorControl.hpp"
#include <Arduino.h>

#define STEP_PIN    0
#define DIR_PIN     1

#define ENABLE_PIN  0
#define INPUT1      1
#define INPUT2      2

Motor::Motor() : current_angle(0.0f), m_pins(nullptr), pin_size(0), motor_type(motor_type), direction(MOTOR_DIRECTION_NONE)
{   }

 bool Motor::Begin(unsigned int encoder_loc, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type)
 {
    this->m_pins = pins; 
    this->pin_size = pin_arr_size; 
    this->motor_type = motor_type;

    for(unsigned int i = 0; i < pin_arr_size; ++i)  
        pinMode(m_pins[i], OUTPUT);

 }

void Motor::MoveTo()
{   
   
        int pmw = map(joystic_analog_val, 0, 800, 0,255);
        analogWrite(m_pins[ENABLE_PIN], pmw); //speed

        while (current_angle != target_angle)
        {
            digitalWrite(m_pins[INPUT1], (direction == MOTOR_DIRECTION_FORWARD)? HIGH : LOW);
            digitalWrite(m_pins[INPUT2], (direction == MOTOR_DIRECTION_FORWARD)? LOW : HIGH);
        }

        //turn off
        digitalWrite(m_pins[INPUT1], LOW);
        digitalWrite(m_pins[INPUT2], LOW);

    
}

double Motor::Calculate_ArcLength(float theta, float radius)
{
    float rad = theta * (M_PI / 180);

    return rad * radius;
}

double Motor::Calculate_Speed(float radius)
{
    float acceleration, velocity, distance;
    distance = Calculate_ArcLength(target_angle, radius);

    float time = 1/500; //time of delay from low to high state of stepper motor
    velocity = distance / time;

    acceleration = velocity/radius;
    return map(acceleration, 500, 0, 50, 1500);
}
#include "MotorControl.hpp"
#include <Arduino.h>
#include <Wire.h>

#define TCA9548A 0x70 
/*
    MOTORTYPE_NORMAL
    index 0 = Enable pin; index 1 = input1; index 2 = input2; all set to output
*/ 
#define ENABLE_PIN  0
#define INPUT1       1
#define INPUT2      2

Motor::Motor() : current_angle(0.0f), m_pins(nullptr), pin_size(0), motor_type(motor_type)
{   }

bool Motor::Begin(unsigned int encoder_loc, float max_angle,const uint32_t *pins, size_t pin_arr_size, MotorType motor_type)
{
    this->m_pins = pins; 
    this->pin_size = pin_arr_size; 
    this->motor_type = motor_type;
    this->encoder_loc = encoder_loc;
    this->max_angle = max_angle;

    if(this->motor_type == MOTORTYPE_NORMAL)
    {
        feedback_loop.SetMode(AUTOMATIC);
        feedback_loop.SetSampleTime(1);
        feedback_loop.SetOutputLimits(-125, 125);
    }

    for(unsigned int i = 0; i < pin_arr_size; ++i)  
        pinMode(m_pins[i], OUTPUT);

    I2C_Multiplexer(this->encoder_loc);
    encoder2.Initialize_Sensor();
}

void Motor::I2C_Multiplexer(uint8_t serial_bus)
{ 
    if(serial_bus > 7) return;

    Wire.beginTransmission(TCA9548A);
    Wire.write( 1 << serial_bus);
    Wire.endTransmission();
}

void Motor::MoveTo()
{
    if(target_angle == NULL)
    {
        Serial.println("Set_Target_Angle fucntion was not called before this fucntion: MoveTo");
        return;
    }
    I2C_Multiplexer(this->encoder_loc);
    current_angle = encoder2.GetAngle();

    if(motor_type == MOTORTYPE_NORMAL)
    {
        feedback_loop.Compute();
        float speed = fabs(output);
        if(speed > 255)
            speed = 255;
        
        int dir = MOTOR_DIRECTION_FORWARD;
        if(output < 0)
            dir = MOTOR_DIRECTION_REVERSE;
        
        setMotor(dir, speed, m_pins[ENABLE_PIN], m_pins[INPUT1], m_pins[INPUT2]);
    }
    else 
    {
        setStepper
    }
    
}

void Motor::setMotor(unsigned int dir, int pmwVal, uint8_t pmw_pin, uint8_t pin1, uint8_t pin2)
{
    analogWrite(pmw_pin, abs(pmwVal));
    if(dir == MOTOR_DIRECTION_FORWARD)
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    }
    else if(dir == MOTOR_DIRECTION_REVERSE)
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    }
    else
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
}

void Motor::setStepper(unsigned int dir, double angle, uint8_t step_pin, uint8_t dir_pin)
{
    const float Step_moment_angle = 1.8; //full step mode angle
    unsigned int n;
    unsigned int num_steps;
    current_angle = encoder2.GetAngle();
    if(angle != current_angle)
    {
        if(current_angle < angle){
            digitalWrite(dir_pin, HIGH);
            n = angle - current_angle;
            num_steps = n / Step_moment_angle;
        }
        else if(current_angle > angle)
        {
            digitalWrite(dir_pin,LOW);
            n = angle - current_angle;

            if(angle == 0)
                n = current_angle;
            
            num_steps = n / Step_moment_angle;
        }
    }
    
    for(unsigned int i = 0; i < num_steps; i++)
    {
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(1000);
    }
    current_angle = angle;
}

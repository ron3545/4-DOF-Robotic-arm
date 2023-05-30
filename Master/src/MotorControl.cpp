#include "MotorControl.hpp"
#include <Arduino.h>
#include <Wire.h>
#include "PS2.hpp"

#define TCA9548A 0x70 
/*
    MOTORTYPE_NORMAL
    index 0 = Enable pin; index 1 = input1; index 2 = input2; all set to output
*/ 
#define ENABLE_PIN  0
#define INPUT1       1
#define INPUT2      2

//index 0 =  step pin; index 1 = dir pin
#define STEP_PIN 0
#define DIR_PIN  1

Motor::Motor() 
    : encoder_loc(0), current_pos(0.0f), target_pos(0), prevTime(0), currentTime(0), 
      deltaTime(0), prevError(0),errorIntegral(0), errorVal(0), edot(0), m_pins(nullptr), 
      pin_size(0), motor_type(motor_type), Bottom_LimitSwitch(0), Top_LimitSwitch(0)
{
    startTime = millis();
}

bool Motor::Begin(unsigned int encoder_loc, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type)
{
    this->m_pins = pins; 
    this->pin_size = pin_arr_size; 
    this->motor_type = motor_type;
    this->encoder_loc = encoder_loc;

    for(unsigned int i = 0; i < pin_arr_size; ++i)  
        pinMode(m_pins[i], OUTPUT);

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(1);
    pid.SetOutputLimits(-180, 180);

    I2C_Multiplexer(this->encoder_loc);
    encoder.Initialize_Sensor();
}

bool Motor::Begin(unsigned int Bottom_LimitSwitch, unsigned int Top_LimitSwitch, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type)
{
    this->m_pins = pins; 
    this->pin_size = pin_arr_size; 
    this->motor_type = motor_type;
    this->Bottom_LimitSwitch = Bottom_LimitSwitch;
    this->Top_LimitSwitch = Top_LimitSwitch;
    
    for(unsigned int i = 0; i < pin_arr_size; ++i)  
        pinMode(m_pins[i], OUTPUT);

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(1);
    pid.SetOutputLimits(-180, 180);

    pinMode(this->Bottom_LimitSwitch, INPUT);
    pinMode(this->Top_LimitSwitch, INPUT);
     pid.SetSampleTime(50);
    
    I2C_Multiplexer(this->encoder_loc);
    encoder.Initialize_Sensor();
}

void Motor::Tune_PID(double* kp, double* ki, double* kd)
{
   this->Kp = *kp;
   this->Ki = *ki;
   this->Kd = *kd; 
}

void Motor::I2C_Multiplexer(uint8_t serial_bus)
{ 
    if(serial_bus > 7) return;

    Wire.beginTransmission(TCA9548A);
    Wire.write( 1 << serial_bus);
    Wire.endTransmission();
}

float Motor::GetCurrentPos()
{
    I2C_Multiplexer(this->encoder_loc);
    current_pos = map(encoder.GetAngle(), 0, 360, 0, 1023);
    return current_pos;
}

void Motor::GrabRelease(unsigned int button)
{
    const int speed = 50;
    analogWrite(m_pins[ENABLE_PIN], speed);
    
    //grab
    if(button == PS2BTN_SQUARE && digitalRead(Top_LimitSwitch) == HIGH)
    {
        if(digitalRead(Bottom_LimitSwitch) != HIGH)
        {
            digitalWrite(m_pins[INPUT1], HIGH);
            digitalWrite(m_pins[INPUT2], LOW);
        }
        else
        {
            digitalWrite(m_pins[INPUT1], LOW);
            digitalWrite(m_pins[INPUT2], LOW);
        }
    }

    //release
    else if(button == PS2BTN_CIRCLE && digitalRead(Bottom_LimitSwitch) == HIGH)
    {
        if(digitalRead(Top_LimitSwitch) != HIGH)
        {
            digitalWrite(m_pins[INPUT1], HIGH);
            digitalWrite(m_pins[INPUT2], LOW);
        }
        else
        {
            digitalWrite(m_pins[INPUT1], LOW);
            digitalWrite(m_pins[INPUT2], LOW);
        }
    }
    else
    {
        digitalWrite(m_pins[INPUT1], LOW);
        digitalWrite(m_pins[INPUT2], LOW);
    }
}

void Motor::MoveTo(float target_angle, unsigned long current_time)
{
    if(target_angle == NULL)
        return;

    //convert angle to pulse
    target_pos = map(target_angle, 0, 360, 0, 1023);

    I2C_Multiplexer(this->encoder_loc);
    current_pos = GetCurrentPos();
    pid.Compute();
    
    Serial.print(">target_pos:");
    Serial.println(target_pos);

    Serial.print(">output:");
    Serial.println(Output);

    Serial.print(">current_pos:");
    Serial.println(current_pos);

    float speed = fabs(Output);
    
    if(speed > 1023)
        speed = 1023;

    int dir = MOTOR_DIRECTION_FORWARD;
    if(Output < 0)
        dir = MOTOR_DIRECTION_REVERSE;
        
    if(motor_type == MOTORTYPE_NORMAL)
        setMotor(dir, speed, m_pins[ENABLE_PIN], m_pins[INPUT1], m_pins[INPUT2]);
    else 
        setStepper(dir, target_angle, m_pins[STEP_PIN], m_pins[DIR_PIN]);
}

void Motor::setMotor(unsigned int dir, int pmwVal, uint8_t pmw_pin, uint8_t pin1, uint8_t pin2)
{
    //ToDo change pwm for joint3
    if(dir ==  MOTOR_DIRECTION_FORWARD)
    {
        float pwm = map(pmwVal, 60, 200, 0, 255);
        analogWrite(pmw_pin, (encoder_loc == JOINT2)?  pwm:  pmwVal);
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    }
    else if(dir == MOTOR_DIRECTION_REVERSE)
    {
        float pwm = map(pmwVal, 990, 1023, 50, 255);
        analogWrite(pmw_pin, (encoder_loc == JOINT2)? constrain(pwm, 990, 1023): pmwVal); 
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
    if(angle)
    {
       
    }
    
    for(unsigned int i = 0; i < num_steps; i++)
    {
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(1000);
    }
    
}

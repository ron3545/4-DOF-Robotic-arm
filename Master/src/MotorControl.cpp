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
    : encoder_loc(0), current_angle(0.0f), target_angle(0), prevTime(0), currentTime(0), 
      deltaTime(0), prevError(0),errorIntegral(0), errorVal(0), edot(0), m_pins(nullptr), 
      pin_size(0), motor_type(motor_type), Bottom_LimitSwitch(0), Top_LimitSwitch(0)
{}

bool Motor::Begin(unsigned int encoder_loc, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type)
{
    this->m_pins = pins; 
    this->pin_size = pin_arr_size; 
    this->motor_type = motor_type;
    this->encoder_loc = encoder_loc;

    for(unsigned int i = 0; i < pin_arr_size; ++i)  
        pinMode(m_pins[i], OUTPUT);

    Setpoint = -40;
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

    pinMode(this->Bottom_LimitSwitch, INPUT);
    pinMode(this->Top_LimitSwitch, INPUT);
}


void Motor::I2C_Multiplexer(uint8_t serial_bus)
{ 
    if(serial_bus > 7) return;

    Wire.beginTransmission(TCA9548A);
    Wire.write( 1 << serial_bus);
    Wire.endTransmission();
}

float Motor::GetCurrentAngle()
{
    I2C_Multiplexer(this->encoder_loc);
    current_angle = encoder.GetAngle();
    return current_angle;
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

void Motor::MoveTo(float target_angle)
{
    if(target_angle == NULL)
        return;
    
    I2C_Multiplexer(this->encoder_loc);

    current_angle = encoder.GetAngle();
    Output = PID_Controller(target_angle, current_angle);

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
    
    current_angle = encoder.GetAngle();
}

void Motor::setMotor(unsigned int dir, int pmwVal, uint8_t pmw_pin, uint8_t pin1, uint8_t pin2)
{
    //ToDo change pwm for joint3
    if(dir == MOTOR_DIRECTION_FORWARD)
    {
        analogWrite(pmw_pin, (encoder_loc == JOINT2)? map(pmwVal, 60, 200, 0, 255) :  map(pmwVal, 990, 1023, 50, 255));
        digitalWrite(pin1,  HIGH);
        digitalWrite(pin2, LOW);
    }
    else if(dir == MOTOR_DIRECTION_REVERSE)
    {
        analogWrite(pmw_pin, (encoder_loc == JOINT2)? map(pmwVal, 990, 1023, 50, 255) : map(pmwVal, 990, 1023, 50, 255)); 
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

float  Motor::PID_Controller(float set_point, float current)
{   
    float proportional = 0.00f;
    float integral = 0.00f;    
    float derivative = 0.00f;

    if(encoder_loc == JOINT2)
    {
        proportional = 0.5; 
        integral     = 0.00333; 
        derivative   = 0.048223;    
    }
    else
    {
        proportional = 3.7; 
        integral     = 0.000433; 
        derivative   = 0.059223;
    }

    currentTime = micros();
    deltaTime = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    errorVal =  current - set_point;
    edot = (errorVal - prevError) / deltaTime;

    errorIntegral = errorIntegral + (errorVal * deltaTime);
    double control_signal = (proportional * errorVal) + (derivative * edot) + (integral *errorIntegral);

    prevError = errorVal;

    //sendToPC(&target_angle, &current_angle);
    return control_signal;
}


void Motor::sendToPC(int* data)
{
    byte* byteData = (byte*)(data);    // Casting to a byte pointer
    Serial.write(byteData, 2);         // Send through Serial to the PC
}

void Motor::sendToPC(double* data)
{
    byte* byteData = (byte*)(data);    // Casting to a byte pointer
    Serial.write(byteData, 4);         // Send through Serial to the PC
}

void Motor::sendToPC(double* data1, double* data2)
{
    byte* byteData1 = (byte*)(data1);
    byte* byteData2 = (byte*)(data2);

    byte buf[6] = {byteData1[0], byteData1[1],
                   byteData2[0], byteData2[1]};
    Serial.write(buf, 6);
}

void Motor::sendToPC(int* data1, int* data2)
{
    byte* byteData1 = (byte*)(data1);
    byte* byteData2 = (byte*)(data2);
    byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3]
                   };
    Serial.write(buf, 12);
}
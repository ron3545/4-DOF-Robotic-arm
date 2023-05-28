#pragma once
#include <Arduino.h>
#include <inttypes.h>
#include "Mathematics.hpp"
#include <PID_v1.h>
#include "Encoder.h"
//#include <AS5600.h>

#define HOME_POS_LNK3 360   //degrees
#define MAX_POS_LNK3 191.87 //degrees; to get here the rotation should be Counter Clockwise

#define HOME_POS_LNK2 0.0f  //degrees
#define MAX_POS_LNK2 131.48 //degrees to get here the rotation of the motors should be at Clockwise

//Unit of measurement should be in cm
enum  MotorDirections 
{
    MOTOR_DIRECTION_FORWARD =  1,
    MOTOR_DIRECTION_REVERSE = -1
};

enum MotorType
{
    MOTORTYPE_STEPPER,
    MOTORTYPE_NORMAL
};

//important 
enum ENCODER_LOCATION 
{
    NONE   =   -1,
    BASE   =    1, 
    JOINT2 =    0,
    JOINT3 =    2
};

class Motor
{
private:
    unsigned int encoder_loc;
    double current_angle, target_angle;   
    double prev_pos;
    //PID
    float prevTime, currentTime, deltaTime, prevError, errorIntegral, errorVal, edot;
    volatile double Setpoint, Output;
    const uint32_t* m_pins; size_t pin_size;

    unsigned int Bottom_LimitSwitch, Top_LimitSwitch; // limit switches

    MotorType       motor_type;
    AS5600 encoder;
public:
    Motor();

    bool Begin(unsigned int encoder_loc, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);
    bool Begin(unsigned int Bottom_LimitSwitch, unsigned int Top_LimitSwitch, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);

    void GrabRelease(unsigned int button);
    void MoveTo(float target_angle);

    float GetCurrentAngle();
    float PID_Controller(float set_point, float current);

    //Serial plotter
    void sendToPC(int* data);
    void sendToPC(double* data);
    void sendToPC(int* data1, int* data2);
    void sendToPC(double* data1, double* data2);
private:
    void setMotor(unsigned int dir, int pmwVal, uint8_t pmw_pin, uint8_t pin1, uint8_t pin2);
    void setStepper(unsigned int dir, double target_position,uint8_t step_pin, uint8_t dir_pin);

    void I2C_Multiplexer(uint8_t serial_bus);
};
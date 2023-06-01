#pragma once
#include <Arduino.h>
#include "Mathematics.hpp"
#include "Encoder.h"
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

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
    JOINT3 =    2,
    JOINT4 =    3
};


void read_encoder();

class Motor
{
protected:
    unsigned int encoder_loc;
    double current_pos, target_pos;   
    double prev_pos;

    byte lastbutton_State = 0;

    unsigned long startTime1, startTime2;

    const long interval1 = 18, interval2 = 20;
    //PID
  
    double Output;
    const uint32_t* m_pins; size_t pin_size;
    const uint32_t  enca, encb;

    unsigned int Bottom_LimitSwitch, Top_LimitSwitch; // limit switches

    //PID
    const double Kp = 1.5, Ki = 0.0141, Kd = 0;
    PID pid = PID(&current_pos, &Output, &target_pos, Kp, Ki, Kd, REVERSE);
    
    MotorType motor_type;
    AS5600 encoder;

public:
    Motor();

    bool Begin(unsigned int encoder_loc, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);
    bool Begin(unsigned int encoder_loc, const uint32_t enca, const uint32_t encb,const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);
    bool Begin(unsigned int Bottom_LimitSwitch, unsigned int Top_LimitSwitch, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);

    void GrabRelease( bool should_grab, unsigned long currentTime);
    void MoveTo(float target_angle);
    void MoveTo(float target_angle, int current_position);

    float GetAngle();
    float GetCurrentPos();
private:
    void setMotor(unsigned int dir, int pmwVal, uint8_t pmw_pin, uint8_t pin1, uint8_t pin2);
    void setStepper(unsigned int dir, double target_position,uint8_t step_pin, uint8_t dir_pin);
    void I2C_Multiplexer(uint8_t serial_bus);
    static void read_encoder();
};
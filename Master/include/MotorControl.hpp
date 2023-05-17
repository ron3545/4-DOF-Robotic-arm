#pragma once
#include <Arduino.h>
#include <inttypes.h>
#include "Mathematics.hpp"
#include <PID_v1.h>
#include "Encoder.h"

//motor at joint 4 has a ratio of 1 : 34
#define JOINT4_MOTOR_1_REVOLUTION 34 

//motors at joint2 and 3 has a ratio of  1 : 48
#define JOINT2_n_3_MOTOR_1_REVOLUTION 48

//Unit of measurement should be in cm
enum  MotorDirections 
{
    MOTOR_DIRECTION_FORWARD = 1,
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
    BASE   =    1, 
    JOINT1 =    0,
    JOINT2 =    2
};

class Motor
{
private:
    unsigned int encoder_loc;

    float max_angle; //max deg of motion
    double kp = 1, ki = 0.025, kd = 0;
    double current_angle = 0, target_angle = 0;   
    double output = 0; //control signal

    unsigned int joint_type;
    const uint32_t* m_pins; size_t pin_size;

    MotorType       motor_type;

    PID feedback_loop = PID(&current_angle, &output, &target_angle, kp, ki, kd, DIRECT);
    
    Magnetic_Encoder encoder1; //facilitates home operation
    AS5600 encoder2; //facilitates run operation
public:
    //initialization of pins happens here
    Motor();
    
    /*
        initiates encoders
        encoder_loc is important since it is the index to the bus on whic the sensor is located

    */
    bool Begin(unsigned int encoder_loc, float max_angle,const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);

    void Set_Joint_Type(unsigned int joint_type)            { this->joint_type =  joint_type; }
   
   //setpoint for the PID; Call this before MoveTo function;
    void Set_Target_Position(float target_angle)            { this->target_angle = target_angle; }

   void MoveTo();

private:
    void setMotor(unsigned int dir, int pmwVal, uint8_t pmw_pin, uint8_t pin1, uint8_t pin2);
    //can be exclueded to PID controll
    void setStepper(unsigned int dir, double target_position,uint8_t step_pin, uint8_t dir_pin);

    void I2C_Multiplexer(uint8_t serial_bus);
};
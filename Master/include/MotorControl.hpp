#pragma once
#include <Arduino.h>
#include <inttypes.h>
#include "Mathematics.hpp"

//Unit of measurement should be in cm
enum  MotorDirections 
{
    MOTOR_DIRECTION_NONE    = 0,
    MOTOR_DIRECTION_FORWARD = 1,
    MOTOR_DIRECTION_REVERSE = 2
};

enum MotorType
{
    MOTORTYPE_STEPPER,
    MOTORTYPE_NORMAL
};

class Motor
{
private:
    float current_angle, target_angle;
    float joystic_analog_val;
    float radius;    

    const uint32_t* m_pins; size_t pin_size;

    MotorType       motor_type;
    MotorDirections direction;
    End_Effector    end_effector;

    unsigned int joint_type;
public:
    //initialization of pins happens here
    Motor();
    
    //this will initialize the encoders
    bool Begin(unsigned int encoder_loc, const uint32_t *pins, size_t pin_arr_size, MotorType motor_type);

    void Set_speed(float speed)                { this->joystic_analog_val = joystic_analog_val; }
    void Set_Joint_Type(unsigned int joint_type)            { this->joint_type =  joint_type; }
    void SetEnd_Effector(End_Effector end_effector)         { this->end_effector = end_effector; }
    void SetDirection(const MotorDirections& direction)     { this->direction = direction;}
    void Set_Target_Angle(float target_angle)               { this->target_angle = target_angle; }

    //this will set the radius on which the arm is able to move
    void Set_Radius(float radius)                           { this->radius = radius; }

    void MoveTo();  

private:
    float Get_CurrentAngle();
    float convertRawAngleToDegrees(unsigned int newAngle);
    double Calculate_Speed(float radius);
    double Calculate_ArcLength(float theta, float radius);
};
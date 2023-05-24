/*
  Author: Ron Joshua S. Quirap

  This is an implementation for the master arduino.
  The task of the master is to control the motors, gather data from the slave sensors, and request 
  data from the arduino slave for the kinematic model.
  
*/
#include "PS2.hpp"
#include "MotorControl.hpp"
#include "Encoder.h"

#include <string.h>
#include <Wire.h>
#include <stdarg.h>

#define ARRAY_SIZE(arr) sizeof(arr) / sizeof(arr[0])

#define NJOINTS 2

//=====================boolean==========================
bool Failed_Initialization = false;
bool is_all_encoders_initiated = false;

//=====================class============================
PS2 ps2;
InverseKinematics_4DOF IK;

Motor motors[NJOINTS];
Motor Joint2;
//=====================struct===========================
static End_Effector end_effector;
static Joint_Angle  joint_angles;
//=====================variables========================
PS2::PS2DATA ps2data;
int PS2error;

float current_angle[NJOINTS] = {0};
float target_angle[NJOINTS]  = {0};

unsigned long startTime_PS2           = millis(); 
unsigned long startTime_MotorControl  = millis();
unsigned long startTime_PrintAngles   = millis(); 

//====================constants=========================
//time intervals
const long interval_ps2           = 2;    
const long interval_motor         = 1;
const long interval_print_angles  = 1000; 

const float LENGTH_LINK2      = 16.51;  //cm
const float LENGTH_LINK3_TO_4 = 21.59;  //cm
const float HEIGHT_BASE       = 3.81;   //cm
const float HEIGHT_LINK1      = 10.922; //cm

/*
    MOTORTYPE_NORMAL
    index 0 = Enable pin; index 1 = input1; index 2 = input2;
*/ 
const size_t size = 3;
const uint32_t motor_pins[4][size] = {
      {13, 50, 52}, //motor 1
      {12, 24, 22}, //motor 2
      {11, 48, 44}, //motor 3
      {10, 32, 34}  //motor 4 
};

//stepper motor
//index 0 =  step pin; index 1 = dir pin
const uint32_t stepper_pins[] = {7, 4};

unsigned int current_raw_angle[NJOINTS];

//======================================functions======================================
static void ReadPS2();
//======================================main===========================================

void setup()
{ 
  IK = InverseKinematics_4DOF(LENGTH_LINK2, LENGTH_LINK3_TO_4, HEIGHT_LINK1, HEIGHT_BASE);
  
  Joint2.Begin(JOINT2, motor_pins[0], 3, MOTORTYPE_NORMAL);

  const unsigned int type[NJOINTS] = {JOINT2, JOINT3}; 
  const float max_angle[NJOINTS] = {MAX_POS_LNK2, MAX_POS_LNK3};
  //for(int i = 0; i < NJOINTS; ++i) 
  //{
    //Motor tmp;
    //tmp.Begin(type[i], max_angle[i], motor_pins[i], 3, MOTORTYPE_NORMAL);
    //motors[i] = tmp;
  //}

  Serial.begin(9600);
  Wire.begin(); 

  const uint8_t Data      = 6;
  const uint8_t Cmd       = 5;
  const uint8_t Attention = 4;
  const uint8_t Clock     = 7;
  ps2.setupPins(Data, Cmd, Attention, Clock, interval_ps2);
  ps2.config(PS2MODE_ANALOG);

  //set the current or home position of end effector
  end_effector.x = 5;   //cm
  end_effector.y = 0;    //cm
  end_effector.z = 24.35;  //cm
}

void loop() 
{ 
  /*
    task 1: read th ps2 controller
    task 2: calculate IK
    task 3: checks the current angle of each of the motor
    task 4: move the motor to pos. This takes time of 1 microseconds, when using stepper motor
    task 5: print the current angles for each motors every 1 seconds
  */
  unsigned long currentTime = millis();
  
  //task1
  if(currentTime - startTime_PS2 > interval_ps2)
  {
    ReadPS2();
    startTime_PS2 = currentTime;
  }
  //task2
  IK.Calculate_Joint_Angles(end_effector, joint_angles);

#pragma region TASK3
{
  //float angles [NJOINTS] = {joint_angles.theta2, joint_angles.theta3};
  //for(unsigned long i = 0; i < NJOINTS; ++i)
  //{
    //current_angle[i] = motors[i].GetCurrentAngle();
    //motors[i].MoveTo(angles[i]);
  //}

  Joint2.MoveTo(joint_angles.theta2);
  current_angle[0] = Joint2.GetCurrentAngle();
}
#pragma endregion

#pragma region TASK4
{
  if(currentTime - startTime_MotorControl >= interval_print_angles){
    Serial.print("Target Angles -> Base: " + String(joint_angles.theta1) + " Joint 2: " + String(joint_angles.theta2) + " Joint 3: " + String(joint_angles.theta3));
    Serial.print("\t|\tCurrent Angles -> Joint2: " + String(current_angle[0]) + " Joint 3: " + String(current_angle[1]));
    Serial.println(" ");
    startTime_PrintAngles = currentTime;
  }
}
#pragma endregion
  
}
//======================================functions impl====================================

void ReadPS2()
{
  const unsigned int JoyY_mid = 132;
  const unsigned int JoyX_mid = 123;

  const float MAX_X_COORDINATE  = 29;
  const float MAX_Y_COORDINATE  = 28.10;
  const float MAX_Z_COORDINATE  = 24 ;

  const float MIN_Y_COORDINATE  = -15.90;

  const float val = 2;

  PS2error = ps2.read(ps2data);
  if(PS2error == PS2ERROR_SUCCESS) 
  {
      unsigned int X = ps2data.JoyLeftX;
      unsigned int Y = ps2data.JoyLeftY;
      unsigned int Z = ps2data.JoyRightY;
      
      if(X >= 0 && X < JoyX_mid)
      {
        if(end_effector.x > MAX_X_COORDINATE)
          end_effector.x = MAX_X_COORDINATE;
        else
          end_effector.x += val;
      }
      else if(X > JoyX_mid)
      {
        if(end_effector.x < 0.0f)
          end_effector.x = 5.0f;
        else 
          end_effector.x -= val;
      }

      if(Y >= 0 && Y < JoyY_mid)
      {
        if(end_effector.y > MAX_Y_COORDINATE)
          end_effector.y = MAX_Y_COORDINATE - 0.5;
        else
          end_effector.y += val;
      }
      else if(Y > JoyY_mid)
      {
        if(end_effector.y < MIN_Y_COORDINATE)
          end_effector.y = MIN_Y_COORDINATE;
        else  
          end_effector.y -= val;
      }

      if(Z >= 0 && Z < JoyY_mid)
      {
        if(end_effector.z > MAX_Z_COORDINATE)
          end_effector.z = MAX_Z_COORDINATE;
        else 
          end_effector.z += val;
      }
      else if(Z > JoyY_mid)
      {
        if(end_effector.z < 0.00f)
          end_effector.z = 2.00f;
        else 
          end_effector.z -= val;
      }
  }
}



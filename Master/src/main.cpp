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

Motor Joint1; //base
Motor Joint2;
Motor Joint3;
Motor Joint4;
//=====================struct===========================
static End_Effector end_effector;
static Joint_Angle  joint_angles;
//=====================variables========================
PS2::PS2DATA ps2data;
int PS2error;

float current_angle[NJOINTS] = {0};
float target_angle[NJOINTS]  = {0};

unsigned long startTime_PS2           = millis(); 
unsigned long startTime_Motor1        = millis();
unsigned long startTime_Motor2        = millis();
unsigned long startTime_Motor3        = millis();
unsigned long startTime_Motor4        = millis();
unsigned long startTime_PrintAngles   = millis(); 

unsigned long timer = micros();
long loopTime = 5000;  
//====================constants=========================
//time intervals
const long interval_ps2           = 2;    
const long interval_motor1        = 10;
const long interval_motor2        = 12;
const long interval_motor3        = 14;
const long interval_motor4        = 16;
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
//index 0 = dir pin; index 1 = step pin
const uint32_t stepper_pins[] = {27, 26};
//======================================functions======================================
static void ReadPS2();
void timeSync(unsigned long deltaT);

static void Move_Motor1(unsigned long current_time, float setPoint);
static void Move_Motor2(unsigned long current_time, float setPoint);
static void Move_Motor3(unsigned long current_time, float setPoint);
static void Move_Motor4(unsigned long current_time, float setPoint);
//======================================main===========================================

void setup()
{ 
  Serial.begin(9600);
  Wire.begin(); 

  IK = InverseKinematics_4DOF(LENGTH_LINK2, LENGTH_LINK3_TO_4, HEIGHT_LINK1, HEIGHT_BASE);
  
  Joint2.Begin(JOINT2, motor_pins[0], 3, MOTORTYPE_NORMAL);
  //Joint3.Begin(JOINT3, motor_pins[1], 3, MOTORTYPE_NORMAL);

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
  unsigned long currentTime = millis();
  
#pragma once Task1
  if(currentTime - startTime_PS2 > interval_ps2)
  {
    ReadPS2();
    startTime_PS2 = currentTime;
  }
#pragma endregion

  IK.Calculate_Joint_Angles(end_effector, joint_angles);

#pragma region TASK3
{
  Move_Motor2(currentTime, 85);
  //Move_Motor3(currentTime, 90);
}
#pragma endregion

#pragma region TASK4
{
  if(currentTime - startTime_PrintAngles >= interval_print_angles){
    Serial.print("Target Angles -> Joint1/Base: " + String(joint_angles.theta1) + " Joint 2: " + String(joint_angles.theta2) + " Joint 3: " + String(joint_angles.theta3));
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

void Move_Motor1(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor1 > interval_motor1)
  {
    startTime_Motor1 = current_time;
  }
}
void Move_Motor2(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor2 > interval_motor2)
  {
    Joint2.MoveTo(setPoint);
    current_angle[0] = Joint2.GetCurrentAngle();

    startTime_Motor2 = current_time;
  }
}
void Move_Motor3(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor3 > interval_motor3){
    Joint3.MoveTo(setPoint);
    current_angle[1] = Joint3.GetCurrentAngle();

    startTime_Motor3 = current_time;
  }
}
void Move_Motor4(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor4 > interval_motor4)
  {
    startTime_Motor4 = current_time;
  }
}

//serial plotter
void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}
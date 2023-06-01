#include "PS2.hpp"
#include "MotorControl.hpp"
#include "Encoder.h"

#include <string.h>
#include <Wire.h>

#define ARRAY_SIZE(arr) sizeof(arr) / sizeof(arr[0])

#define NJOINTS 2

//=====================controller variables=============

int prevXValue = 0;  // Previous X-axis joystick value
int prevYValue = 0;  // Previous Y-axis joystick value
int prevZValue = 0;  // Previous Z-axis joystick value

//=====================class============================
Motor Joint1; //base
Motor Joint2;
Motor Joint3;
Motor Joint4;

Motor Gripper;
//=====================struct===========================
static End_Effector end_effector;
static Joint_Angle  joint_angles;
//=====================variables========================
int pos = 0;
float joint1_angle = 0.0f, joint2_angle = 0.0f, joint3_angle = 0.0f, joint4_angle = 0.0f;

bool should_grab = false;
byte last_button_state;

unsigned long startTime_readPot1       = millis();
unsigned long startTime_readPot2       = millis();
unsigned long startTime_readPot3       = millis();
unsigned long startTime_readPot4       = millis();

unsigned long startTime_button        = millis();

unsigned long startTime_Motor1        = millis();
unsigned long startTime_Motor2        = millis();
unsigned long startTime_Motor3        = millis();
unsigned long startTime_Motor4        = millis();
unsigned long startTime_PrintAngles   = millis(); 

unsigned long timer = micros();
long loopTime = 5000;  
//====================constants=========================
//time intervals
const long interval_Pot1           = 1; 
const long interval_Pot2           = 2; 
const long interval_Pot3           = 3; 
const long interval_Pot4           = 4; 

const long interval_button        = 5; 

const long interval_motor1        = 10;
const long interval_motor2        = 12;
const long interval_motor3        = 14;
const long interval_motor4        = 16;
 

const uint32_t hall_sensor1 = 40, hall_sensor2 = 42;
const uint32_t Grab_Release = A4, pot_joint4 = A3, pot_joint3 = A2, pot_joint2 = A1, pot_joint1 = A0;
/*
    MOTORTYPE_NORMAL
    index 0 = Enable pin; index 1 = input1; index 2 = input2;
*/ 
const size_t size = 3;
const uint32_t motor_pins[4][size] = {
      {13, 50, 52}, //motor 1
      {12, 24, 22}, //motor 2
      {10, 32, 30}, //motor 3
      {11, 44, 46}  //motor 4 
};
//                 top                bottom
const uint32_t limit_switch1 = 38, limit_switch2 = 36;
//stepper motor
//index 0 = dir pin; index 1 = step pin
const uint32_t stepper_pins[] = {27, 26};
//======================================functions======================================
static void Read_Values(unsigned long currentTime);
static void Move_Motor1(unsigned long current_time, float setPoint);
static void Move_Motor2(unsigned long current_time, float setPoint);
static void Move_Motor3(unsigned long current_time, float setPoint);
static void Move_Motor4(unsigned long current_time, float setPoint);
void read_encoder();
//======================================main===========================================

void setup()
{ 
  Serial.begin(9600);
  Wire.begin(); 
  
  pinMode(hall_sensor1, INPUT); 
  pinMode(hall_sensor2, INPUT);

  pinMode(limit_switch1, INPUT_PULLUP);
  pinMode(limit_switch2, INPUT_PULLUP);

  pinMode(Grab_Release, INPUT_PULLUP);

  Joint1.Begin(BASE, stepper_pins, 2, MOTORTYPE_STEPPER);
  Joint2.Begin(JOINT2, motor_pins[0], 3, MOTORTYPE_NORMAL);
  Joint3.Begin(JOINT3, motor_pins[1], 3, MOTORTYPE_NORMAL);
  Joint4.Begin(JOINT4, hall_sensor1, hall_sensor2, motor_pins[3], 3, MOTORTYPE_NORMAL);

  Gripper.Begin(limit_switch2, limit_switch1, motor_pins[2], 3, MOTORTYPE_NORMAL);

  last_button_state = !digitalRead(Grab_Release);

  attachInterrupt(digitalPinToInterrupt(hall_sensor1), read_encoder, RISING);

}

void loop() 
{ 
  unsigned long currentTime = millis();
  
  Read_Values(currentTime);
  Move_Motor2(currentTime, joint2_angle);
  Move_Motor3(currentTime, joint3_angle);
}
//======================================functions impl====================================

void read_encoder()
{
  int b = digitalRead(hall_sensor2);
  if(b > 0) pos++;
  else pos--;
}
void Read_Values(unsigned long currentTime)
{
  if(currentTime - startTime_readPot1 > interval_Pot1)
  {
    joint1_angle = map(analogRead(pot_joint1), 0, 1023, -90, 90);
    startTime_readPot1 = currentTime;
  }

  if(currentTime - startTime_readPot2 > interval_Pot2)
  {
    joint2_angle = map(analogRead(pot_joint2), 0, 1023, -20, 120);
    startTime_readPot2 = currentTime;
  }

  if(currentTime - startTime_readPot3 > interval_Pot3)
  {
    joint3_angle = map(analogRead(pot_joint3), 0, 1023, -120, 120);
    startTime_readPot3 = currentTime;
  }

  if(currentTime - startTime_readPot4 > interval_Pot4)
  {
    joint4_angle = map(analogRead(pot_joint4), 0, 1023, -90, 90);
    startTime_readPot4 = currentTime;
  }

 
}

void Move_Motor1(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor1 > interval_motor1)
  {
    Joint1.MoveTo(setPoint);
    startTime_Motor1 = current_time;
  }
}
void Move_Motor2(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor2 > interval_motor2)
  {
    Joint2.MoveTo(setPoint);
    startTime_Motor2 = current_time;
  }
}
void Move_Motor3(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor3 > interval_motor3){
    Joint3.MoveTo(setPoint);
    startTime_Motor3 = current_time;
  }
}
void Move_Motor4(unsigned long current_time, float setPoint)
{
  if(current_time - startTime_Motor4 > interval_motor4)
  {
    Joint3.MoveTo(setPoint, pos);
    startTime_Motor4 = current_time;
  }
}


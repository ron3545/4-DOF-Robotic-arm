/*
  Author: Ron Joshua S. Quirap

  This is an implementation for the master arduino.
  The task of the master is to control the motors, gather data from the slave sensors, and request 
  data from the arduino slave for the kinematic model.
  
*/
#include <Arduino.h>
#include "PS2.hpp"
#include "MotorControl.hpp"
#include "Encoder.h"

#include <string.h>
#include <Wire.h>
#include <stdarg.h>

#define ARDUINO_SLAVE_ADDR  0x11

#define CALCULATE_ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#define MAX_LENGTH(...) MaxLength(__VA_ARGS__);
#define RADIUS(maxlength) map(end_effector.x, 0, 255, 0, maxlength);
#define ARRAY_SIZE(arr) sizeof(arr) / sizeof(arr[0])

#define NJOINTS 4

//=====================home angles======================
#define HOME_FOR_JOINT_3 191.78
#define HOME_FOR_JOINT_2 132.36

//=====================MAX angles======================
#define JOINT1_MAX_ANGLE  -174.11
#define JOINT2_MAX_ANGLE  194.12
//=====================boolean==========================
bool Should_Exit = false;
bool Failed_Initialization = false;
bool is_all_encoders_initiated = false;

//=====================class============================
Motor           * motors        = nullptr;

//=====================struct===========================
End_Effector end_effector;

//=====================variables========================
 unsigned int buttons; //for ps2 constroller
/*
    MOTORTYPE_NORMAL
    index 0 = Enable pin; index 1 = input1; index 2 = input2;
*/ 
const unsigned int motor_pins[4][3] = {
      {11, A0, A1},
      {10, A2, A3}
};

//stepper motor
const uint8_t stepper_pins[] = 7, Dir_pin = 4;

unsigned int current_raw_angle[NJOINTS];

float current_angle[NJOINTS];
float target_angle[NJOINTS];


//======================================functions=========================================

float MaxLength(size_t n_args, ...);

void I2C_Multiplexer(uint8_t serial_bus)
{ 
    if(serial_bus > 7) return;

    Wire.beginTransmission(0x70 );
    Wire.write( 1 << serial_bus);
    Wire.endTransmission();
}
static void Destroy_ptr();

static void Send_Data_to_Arduino_Slave(const float angle[], size_t size);
static void Receive_Data_from_Slave();
void Get_PS2_Button_State();
//======================================main===========================================

Magnetic_Encoder encoder;
int pos = 0;
AS5600 en;


//AS5600 as5600;
void setup()
{
  motors   = new Motor[NJOINTS];

  Serial.begin(9600);
  Wire.begin(); 
  I2C_Multiplexer(2);
  en.Initialize_Sensor();
}


void loop() 
{
  I2C_Multiplexer(2);
  Serial.println(String(en.GetAngle()));
}
//======================================functions impl====================================

//send current motor angles
void Send_Data_to_Arduino_Slave(const float angle[], size_t size)
{
  if(angle == nullptr)  return;
  byte floatdata[sizeof(float) * sizeof(current_angle)];
  memcpy(&floatdata, &current_angle, sizeof(current_angle));

  Wire.beginTransmission(ARDUINO_SLAVE_ADDR);
  Wire.write(floatdata, sizeof(floatdata));
  Wire.endTransmission();
}


void Get_PS2_Button_State()
{
  Wire.requestFrom(ARDUINO_SLAVE_ADDR, 2);
  unsigned int receivedData = 0;
  if (Wire.available() >= 2) {
    byte lowByte = Wire.read();
    byte highByte = Wire.read();
    receivedData = lowByte | (highByte << 8);
    Serial.print("Received data: ");
    Serial.println(receivedData);

    buttons = receivedData;
  }
}

void Receive_Data_from_Slave()
{
  Wire.requestFrom(ARDUINO_SLAVE_ADDR, NJOINTS * sizeof(uint16_t));
  byte angle_floats[sizeof(float) * NJOINTS];

  for(int i = 0; i < sizeof(float) * NJOINTS; ++i)
    angle_floats[i] = Wire.read();
  
  memcpy(&target_angle, angle_floats, sizeof(float) * NJOINTS);
}

float convertRawAngleToDegrees(unsigned int newAngle)
{
  float angle = newAngle * 0.087890625;
  return angle;
}

void Destroy_ptr()
{
  if(motors != nullptr)
    delete[] motors;
}

float MaxLength(size_t n_args, ...)
{
  float max_length = 0;

  va_list args;
  va_start(args, n_args);

  for(size_t i = 0; i < n_args; ++i)
    max_length += va_arg(args, float);
  va_end(args);

  return max_length;
}


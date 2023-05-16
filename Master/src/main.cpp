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

#define TCA9548A 0x70 

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
      {11, 12, 10}, // link2
      {9, 8, 7}     // link3
};

unsigned int current_raw_angle[NJOINTS];

float current_angle[NJOINTS];
float target_angle[NJOINTS];

/*
  index 1 = base
  index 0 = joint 1
  index 2 = joint 2
*/
const unsigned int sensor_index_bus[] = {1, 0, 2};

unsigned int encoder_loc[] = {
  ENCODER_LOCATION_JOINT1,
  ENCODER_LOCATION_JOINT2,
  ENCODER_LOCATION_JOINT3,
  ENCODER_LOCATION_JOINT4,
};

//======================================functions=========================================
static float convertRawAngleToDegrees(unsigned int newAngle);
static float convertScaledAngleToDegrees(unsigned int newAngle);
static String burnAngle();

static float MaxLength(size_t n_args, ...);
static void I2C_Multiplexer(uint8_t serial_bus);
static void Destroy_ptr();
static void GetCurrentRawAngle(unsigned int *raw_angle, float* angle_degrees);

static void Send_Data_to_Arduino_Slave(const float angle[], size_t size);
static void Receive_Data_from_Slave();
static void Get_PS2_Button_State();
//======================================main===========================================

Magnetic_Encoder encoder;
//AS5600 as5600;
void setup()
{
  motors   = new Motor[NJOINTS];

  Serial.begin(9600);
  Wire.begin();

  SERIAL.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  I2C_Multiplexer(2);
  SERIAL.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  
  if(encoder.detectMagnet() == 0 ){
    while(1){
        if(encoder.detectMagnet() == 1 ){
            SERIAL.print("Current Magnitude: ");
            SERIAL.println(encoder.getMagnitude());
            break;
        }
        else{
            SERIAL.println("Can not detect magnet");
        }
        delay(1000);
    }
  }


  //as5600.Initialize_Sensor();

}


void loop() 
{
   I2C_Multiplexer(2);
   
  SERIAL.println(String(convertRawAngleToDegrees(encoder.getRawAngle()),DEC));
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

void I2C_Multiplexer(uint8_t serial_bus)
{ 
  if(serial_bus > 7) return;

  Wire.beginTransmission(TCA9548A);
  Wire.write( 1 << serial_bus);
  Wire.endTransmission();
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


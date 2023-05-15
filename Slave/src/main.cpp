/*
  Author: Ron Joshua S. Quirap

  Arduino Slave

  Tasks:
    -  Read the ps2/ps3 controller 
    -  Calculate InverseKinematics 
    -  Send the target angles to the master
*/
#include <Arduino.h>
#include <Wire.h>

#include "PS2.hpp"
#include "InverseKinematic.h"

#define ARDUINO_SLAVE_ADDR  0x01
#define ARRAY_SIZE 24

//ARRAY CONFIG
#define STRIDE_JOINT_ANGLES 0
#define STRIDE_END_EFFECTOR_POSITION 

typedef unsigned int uint;
constexpr unsigned int NJOINTS = 4;

float ToReceive[NJOINTS]; //receive the current angles of each motor
float ToSend[NJOINTS]; //target angle for the motors to move

PS2 ps2;
PS2::PS2DATA PS2data;
int PS2error;
uint button;

bool Should_Exit = false;

const uint8_t Data      = 12;
const uint8_t Cmd       = 11;
const uint8_t Attention = 10;
const uint8_t Clock     = 13;

End_Effector end_effector;

static void receiveEvent(int bytes);
static void SendAngles();
static void SendPS2ButtonState(uint button);

void setup()
{
  Serial.begin(9600);
  Wire.begin(ARDUINO_SLAVE_ADDR);

  ps2.setupPins(Data, Cmd, Attention, Clock, 10);
  ps2.config(PS2MODE_ANALOG);
}

void loop()
{
  if(Should_Exit)
    return;

  PS2error = ps2.read(PS2data);
  
  button = PS2data.buttons;
  Wire.onRequest(SendPS2ButtonState);

  // Check if the read was successful
  if(PS2error == PS2ERROR_SUCCESS) {
      // Print the joystick states
      end_effector.x = PS2data.JoyLeftX;
      end_effector.y = PS2data.JoyLeftY;
      end_effector.z = PS2data.JoyRightY;

      if(PS2data.buttons == PS2BTN_CROSS)
        Should_Exit = true;
  }
  
  InverseKinematics::GetInstance().Move_To_Pos(end_effector);
  Linked_List<double> joint_angles = InverseKinematics::GetInstance().Get_Joint_Angles();


}

//receive current angles of the motors
void receiveEvent(int byteCount) {
  byte floatBytes[sizeof(float) * NJOINTS];
  Wire.readBytes(floatBytes, sizeof(float) * NJOINTS);
 
  memcpy(&ToReceive, &floatBytes, sizeof(float) * NJOINTS);
  for (int i = 0; i < NJOINTS; i++) {
    Serial.print("current angles of the motors ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(ToReceive[i]);
  }
}

//send an array of data
void SendAngles() 
{
  byte floatbytes[sizeof(float) * sizeof(ToSend)];
  memcpy(&floatbytes, &ToSend, sizeof(floatbytes));
  Wire.write(floatbytes, sizeof(floatbytes)); // send the data array to the master
}

 void SendPS2ButtonState()
 {
    byte lowByte = button & 0xFF;
    byte highByte = (button >> 8) & 0xFF;
    Wire.write(lowByte);
    Wire.write(highByte);
 }
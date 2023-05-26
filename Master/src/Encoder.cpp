#include "Encoder.h"

#include <Arduino.h>
#include "Wire.h"

/****************************************************
  Method: AMS_5600
  In: none
  Out: none
  Description: constructor class for AMS 5600
*****************************************************/
Magnetic_Encoder::Magnetic_Encoder()
{
}

bool Magnetic_Encoder::infinite_test()
{
  while(1)
  {
    if(detectMagnet() == 1)
    {
      SERIAL.print("Current Magnitude @ " + loc_names[Location] + ": ");
      SERIAL.println(getMagnitude());
      return true;
    }
    else 
    {
      SERIAL.println("Can not detect magnet @" + loc_names[Location]);
    }
    delay(1000);
  }
}

bool Magnetic_Encoder::finite_test(unsigned int max_test)
{
  for(int i = 0; i < max_test; ++i)
  {
    if(detectMagnet() == 1)
    {
      SERIAL.print("Current Magnitude @ " + loc_names[Location] + ": ");
      SERIAL.println(getMagnitude());
      return true;
    }
    else 
    {
      SERIAL.println("Can not detect magnet @" + loc_names[Location]);
    }
    delay(1000);
  }
  return false;
}

bool Magnetic_Encoder::Initiate(unsigned int trial_count)
{
  bool ret;

  if(trial_count == 0)
  {
    if(detectMagnet() == 0)
      ret = infinite_test();
  }
  else 
  {
    if(detectMagnet() == 0)
      ret = finite_test(trial_count);
  }
  
  return ret;
}

/*******************************************************
  Method: setOutPut
  In: 0 for digital PWM
      1 for analog (full range 0-100% of GND to VDD)
      2 for analog (reduced range 10-90%)
  Out: none
  Description: sets output mode in CONF register.
*******************************************************/
void Magnetic_Encoder::setOutPut(uint8_t mode)
{
  int _conf_lo = _addr_conf+1; // lower byte address
  uint8_t config_status = readOneByte(_conf_lo);
  config_status &= 0b11001111; // bits 5:4 = 00, default
  if (mode == 0) {
    config_status |= 0b100000; // bits 5:4 = 10
  } else if (mode == 2) {
    config_status |= 0b010000; // bits 5:4 = 01
  }
  writeOneByte(_conf_lo, config_status);
}

/****************************************************
  Method: AMS_5600
  In: none
  Out: i2c address of AMS 5600
  Description: returns i2c address of AMS 5600
****************************************************/
int Magnetic_Encoder::getAddress()
{
  return _ams5600_Address;
}

/*******************************************************
  Method: setMaxAngle
  In: new maximum angle to set OR none
  Out: value of max angle register
  Description: sets a value in maximum angle register.
  If no value is provided, method will read position of
  magnet.  Setting this register zeros out max position
  register.
*******************************************************/
unsigned int Magnetic_Encoder::setMaxAngle(unsigned int newMaxAngle)
{
  unsigned int _maxAngle;
  if (newMaxAngle == -1)
    _maxAngle = getRawAngle();
  else
    _maxAngle = newMaxAngle;

  writeOneByte(_addr_mang, highByte(_maxAngle));
  delay(2);
  writeOneByte(_addr_mang+1, lowByte(_maxAngle));
  delay(2);

  unsigned int retVal = readTwoBytesSeparately(_addr_mang);
  return retVal;
}

/*******************************************************
  Method: getMaxAngle
  In: none
  Out: value of max angle register
  Description: gets value of maximum angle register.
*******************************************************/
unsigned int Magnetic_Encoder::getMaxAngle()
{
  return readTwoBytesSeparately(_addr_mang);
}

/*******************************************************
  Method: setStartPosition
  In: new start angle position
  Out: value of start position register
  Description: sets a value in start position register.
  If no value is provided, method will read position of
  magnet.  
*******************************************************/
unsigned int Magnetic_Encoder::setStartPosition(unsigned int startAngle)
{
  unsigned int _rawStartAngle;
  if (startAngle == -1)
    _rawStartAngle = getRawAngle();
  else
    _rawStartAngle = startAngle;

  writeOneByte(_addr_zpos, highByte(_rawStartAngle));
  delay(2);
  writeOneByte(_addr_zpos+1, lowByte(_rawStartAngle));
  delay(2);
  unsigned int _zPosition = readTwoBytesSeparately(_addr_zpos);

  return (_zPosition);
}

/*******************************************************
  Method: getStartPosition
  In: none
  Out: value of start position register
  Description: gets value of start position register.
*******************************************************/
unsigned int Magnetic_Encoder::getStartPosition()
{
  return readTwoBytesSeparately(_addr_zpos);
}

/*******************************************************
  Method: setEndPosition
  In: new end angle position
  Out: value of end position register
  Description: sets a value in end position register.
  If no value is provided, method will read position of
  magnet.  
*******************************************************/
unsigned int Magnetic_Encoder::setEndPosition(unsigned int endAngle)
{
  unsigned int _rawEndAngle;
  if (endAngle == -1)
    _rawEndAngle = getRawAngle();
  else
    _rawEndAngle = endAngle;

  writeOneByte(_addr_mpos, highByte(_rawEndAngle));
  delay(2);
  writeOneByte(_addr_mpos+1, lowByte(_rawEndAngle));
  delay(2);
  unsigned int _mPosition = readTwoBytesSeparately(_addr_mpos);

  return (_mPosition);
}

/*******************************************************
  Method: getEndPosition
  In: none
  Out: value of end position register
  Description: gets value of end position register.
*******************************************************/
unsigned int Magnetic_Encoder::getEndPosition()
{
  unsigned int retVal = readTwoBytesSeparately(_addr_mpos);
  return retVal;
}

/*******************************************************
  Method: getRawAngle
  In: none
  Out: value of raw angle register
  Description: gets raw value of magnet position.
  start, end, and max angle settings do not apply
*******************************************************/
unsigned int Magnetic_Encoder::getRawAngle()
{
  return readTwoBytesTogether(_addr_raw_angle);
}

/*******************************************************
  Method: getScaledAngle
  In: none
  Out: value of scaled angle register
  Description: gets scaled value of magnet position.
  start, end, or max angle settings are used to 
  determine value
*******************************************************/
unsigned int Magnetic_Encoder::getScaledAngle()
{
  return readTwoBytesTogether(_addr_angle);
}

float Magnetic_Encoder::GetAngle()
{
  return getRawAngle() * 0.087890625;
}

/*******************************************************
  Method: detectMagnet
  In: none
  Out: 1 if magnet is detected, 0 if not
  Description: reads status register and examines the 
  MD bit.
*******************************************************/
int Magnetic_Encoder::detectMagnet()
{
  // Status bits: 0 0 MD ML MH 0 0 0 
  // MD high = magnet detected  
  int magStatus = readOneByte(_addr_status);
  return (magStatus & 0x20) ? 1 : 0;
}

/*******************************************************
  Method: getMagnetStrength
  In: none
  Out: 0 if magnet not detected
       1 if magnet is too weak
       2 if magnet is just right
       3 if magnet is too strong
  Description: reads status register and examines the 
  MH,ML,MD bits.
*******************************************************/
int Magnetic_Encoder::getMagnetStrength()
{
  int retVal = 0; // no magnet
  // Status bits: 0 0 MD ML MH 0 0 0 
  // MD high = magnet detected  
  // ML high = AGC maximum overflow, magnet too weak
  // MH high = AGC minimum overflow, magnet too strong
  int magStatus = readOneByte(_addr_status);
  if (magStatus & 0x20) {
    retVal = 2;   // magnet detected
    if (magStatus & 0x10)
      retVal = 1; // too weak
    else if (magStatus & 0x08)
      retVal = 3; // too strong
  }
  
  return retVal;
}

/*******************************************************
  Method: get Agc
  In: none
  Out: value of AGC register
  Description: gets value of AGC register.
*******************************************************/
int Magnetic_Encoder::getAgc()
{
  return readOneByte(_addr_agc);
}

/*******************************************************
  Method: getMagnitude
  In: none
  Out: value of magnitude register
  Description: gets value of magnitude register.
*******************************************************/
unsigned int Magnetic_Encoder::getMagnitude()
{
  return readTwoBytesTogether(_addr_magnitude);
}

/*******************************************************
  Method: getConf
  In: none
  Out: value of CONF register 
  Description: gets value of CONF register.
*******************************************************/
unsigned int Magnetic_Encoder::getConf()
{
  return readTwoBytesSeparately(_addr_conf);
}

/*******************************************************
  Method: setConf
  In: value of CONF register
  Out: none
  Description: sets value of CONF register.
*******************************************************/
void Magnetic_Encoder::setConf(unsigned int _conf)
{
  writeOneByte(_addr_conf, highByte(_conf));
  delay(2);
  writeOneByte(_addr_conf+1, lowByte(_conf));
  delay(2);
}

/*******************************************************
  Method: getBurnCount
  In: none
  Out: value of zmco register
  Description: determines how many times chip has been
  permanently written to. 
*******************************************************/
int Magnetic_Encoder::getBurnCount()
{
  return readOneByte(_addr_zmco);
}

/*******************************************************
  Method: burnAngle
  In: none
  Out: 1 success
      -1 no magnet
      -2 burn limit exceeded
      -3 start and end positions not set (useless burn)
  Description: burns start and end positions to chip.
  THIS CAN ONLY BE DONE 3 TIMES
*******************************************************/
int Magnetic_Encoder::burnAngle()
{
  unsigned int _zPosition = getStartPosition();
  unsigned int _mPosition = getEndPosition();
  unsigned int _maxAngle = getMaxAngle();

  int retVal = 1;
  if (detectMagnet() == 1) {
    if (getBurnCount() < 3) {
      if ((_zPosition == 0) && (_mPosition == 0))
        retVal = -3;
      else
        writeOneByte(_addr_burn, 0x80);
    }
    else
      retVal = -2;
  } else
    retVal = -1;

  return retVal;
}

/*******************************************************
  Method: burnMaxAngleAndConfig
  In: none
  Out: 1 success
      -1 burn limit exceeded
      -2 max angle is to small, must be at or above 18 degrees
  Description: burns max angle and config data to chip.
  THIS CAN ONLY BE DONE 1 TIME
*******************************************************/
int Magnetic_Encoder::burnMaxAngleAndConfig()
{
  unsigned int _maxAngle = getMaxAngle();

  int retVal = 1;
  if (getBurnCount() == 0) {
    if (_maxAngle * 0.087 < 18)
      retVal = -2;
    else
      writeOneByte(_addr_burn, 0x40);
  }
  else
    retVal = -1;

  return retVal;
}

/*******************************************************
  Method: readOneByte
  In: register to read
  Out: data read from i2c
  Description: reads one byte register from i2c
*******************************************************/
int Magnetic_Encoder::readOneByte(int in_adr)
{
  int retVal = -1;
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, (uint8_t) 1);
  while (Wire.available() == 0)
    ;
  retVal = Wire.read();

  return retVal;
}

/*******************************************************
  Method: readTwoBytesTogether
  In: two registers to read
  Out: data read from i2c as a unsigned int
  Description: reads two bytes register from i2c
*******************************************************/
unsigned int Magnetic_Encoder::readTwoBytesTogether(int addr_in)
{

  // use only for Angle, Raw Angle and Magnitude

  // read 2 bytes together to prevent getting inconsistent
  //    data while the encoder is moving
  // according to the datasheet the address is automatically incremented
  //    but only for Angle, Raw Angle and Magnitude
  // the title says it's auto, but the paragraph after it
  //    says it does NOT
  // tested and it does auto increment
  
  // PAGE 13: https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
  // Automatic Increment of the Address Pointer for ANGLE, RAW ANGLE and MAGNITUDE Registers
  // These are special registers which suppress the automatic
  // increment of the address pointer on reads, so a re-read of these
  // registers requires no I²C write command to reload the address
  // pointer. This special treatment of the pointer is effective only if
  // the address pointer is set to the high byte of the register.

  /* Read 2 Bytes */
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(addr_in);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, (uint8_t) 2);
  while (Wire.available() < 2)
    ;
  
  int highByte = Wire.read();
  int lowByte  = Wire.read();

  // in case newer version of IC used the same address to
  //    store something else, get only the 3 bits
  //return ( ( highByte & 0b111 ) << 8 ) | lowByte;

  // but in case newer version has higher resolution
  //    we're good to go
  return ( highByte << 8 ) | lowByte;
}

/*******************************************************
  Method: readTwoBytesSeparately
  In: two registers to read
  Out: data read from i2c as a unsigned int
  Description: reads two bytes register from i2c
*******************************************************/
unsigned int Magnetic_Encoder::readTwoBytesSeparately(int addr_in)
{
  int highByte = readOneByte(addr_in  );
  int lowByte  = readOneByte(addr_in+1);
  return ( highByte << 8 ) | lowByte;
}

/*******************************************************
  Method: writeOneByte
  In: address and data to write
  Out: none
  Description: writes one byte to a i2c register
*******************************************************/
void Magnetic_Encoder::writeOneByte(int adr_in, int dat_in)
{
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(adr_in);
  Wire.write(dat_in);
  Wire.endTransmission();
}



//===============================================================================

void AS5600::checkMagnetPresence()
{
   while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  //delay(1000);  
}

void AS5600::checkQuadrant()
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void AS5600::correctAngle()
{
    //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void AS5600::ReadRawAngle()
{
   //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}

float AS5600::GetAngle()
{
  ReadRawAngle(); //ask the value from the sensor
  correctAngle(); //tare the value
  checkQuadrant(); //check quadrant, check rotations, check absolute angular position

  return totalAngle;
}

void AS5600::Initialize_Sensor()
{
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)

  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring
}

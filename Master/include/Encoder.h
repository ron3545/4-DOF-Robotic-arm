#pragma once 
#include <Arduino.h>
#include <string.h>

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

enum Encoder_Location 
{
  ENCODER_LOCATION_JOINT1 = 0,  //Base
  ENCODER_LOCATION_JOINT2 = 1,
  ENCODER_LOCATION_JOINT3 = 2,
  ENCODER_LOCATION_JOINT4 = 3 
};

class Magnetic_Encoder
{
private:
  String loc_names[4] = {"Joint1", "Joint2", "Joint3", "Joint4"};
  unsigned int Location;
public:
  Magnetic_Encoder(void);
  
  void Set_Encoder_loc(unsigned int Location) { this->Location = Location; }

  bool Initiate(unsigned int trial_count = 0);
  int getAddress();

  unsigned int setMaxAngle(unsigned int newMaxAngle = -1);
  unsigned int getMaxAngle();

  unsigned int setStartPosition(unsigned int startAngle = -1);
  unsigned int getStartPosition();

  unsigned int setEndPosition(unsigned int endAngle = -1);
  unsigned int getEndPosition();

  unsigned int getRawAngle();
  unsigned int getScaledAngle();

  int detectMagnet();
  int getMagnetStrength();
  int getAgc();
  unsigned int getMagnitude();
  unsigned int getConf();
  void setConf(unsigned int _conf);

  int getBurnCount();
  int burnAngle();
  int burnMaxAngleAndConfig();
  void setOutPut(uint8_t mode);

private:

  // i2c address
  static const uint8_t _ams5600_Address = 0x36;
  
  // single byte registers
  static const uint8_t _addr_status = 0x0B; // magnet status
  static const uint8_t _addr_agc    = 0x1a; // automatic gain control
  static const uint8_t _addr_burn   = 0xff; // permanent burning of configs (zpos, mpos, mang, conf)
  static const uint8_t _addr_zmco   = 0x00; // number of times zpos/mpos has been permanently burned
                                            // zpos/mpos can be permanently burned 3x
                                            // mang/conf can be burned only once
  
  // double byte registers, specify starting address (lower addr, but higher byte data)
  // addr   = upper byte of data (MSB), only bits 0:3 are used
  // addr+1 = lower byte of data (LSB)
  static const uint8_t _addr_zpos      = 0x01; // zero position (start)
                                               // 0x02 - lower byte
  static const uint8_t _addr_mpos      = 0x03; // maximum position (stop)
                                               // 0x04 - lower byte
  static const uint8_t _addr_mang      = 0x05; // maximum angle
                                               // 0x06 - lower byte
  static const uint8_t _addr_conf      = 0x07; // configuration
                                               // 0x08 - lower byte
  static const uint8_t _addr_raw_angle = 0x0c; // raw angle
                                               // 0x0d - lower byte
  static const uint8_t _addr_angle     = 0x0e; // mapped angle
                                               // 0x0f - lower byte
  static const uint8_t _addr_magnitude = 0x1b; // magnitude of internal CORDIC
                                               // 0x1c - lower byte

  int readOneByte(int in_adr);
  unsigned int readTwoBytesSeparately(int addr_in);
  unsigned int readTwoBytesTogether(int addr_in);
  void writeOneByte(int adr_in, int dat_in);

  //for initializing
  bool infinite_test();
  bool finite_test(unsigned int max_test);
};



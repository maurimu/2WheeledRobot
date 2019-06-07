#ifndef __CODEURS_H__
#define __CODEURS_H__

#include "Arduino.h"

#ifndef RIGHT
  #define RIGHT 1
  #define LEFT 0
#endif

#define PIN_ENC0_A A2  //extint7
#define PIN_ENC0_B A3  //extint0

#define PIN_ENC1_A 11   //extint9
#define PIN_ENC1_B  9  //extint4

void initEncoders();

void resetEncoders();

// encoders resolution: 768 ticks/turn
int32_t getEncoder(uint8_t encoderId);

void getSpeed(int32_t &left, int32_t &right);

#endif
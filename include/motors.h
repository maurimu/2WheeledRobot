#ifndef __MOTEURS_H__
#define __MOTEURS_H__

#include "Arduino.h"
#include "sam.h"

#ifndef RIGHT
#define RIGHT 1
#define LEFT 0
#endif

#define MIN_PWM 85 // below this value the wheels wont turn
#define MAX_PWM 255

#define PIN_RIGHT_MOTOR_PWM_FWD 6  //TC3/WO[0]
#define PIN_RIGHT_MOTOR_PWM_BCK 12 //TC3/WO[1]

#define PIN_LEFT_MOTOR_PWM_FWD 10 //TCC0/WO[6]
#define PIN_LEFT_MOTOR_PWM_BCK 5  //TCC0/WO[3]

void initMotors();

/*
id: LEFT or RIGHT
speed: limits [-255 255], 0 means stop, the sign sets the direction of rotation (forward or backwards)
*/
void setMotor(uint8_t id, int16_t speed);

#endif
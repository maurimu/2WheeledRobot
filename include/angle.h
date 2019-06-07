//eval acceleromètre MPU6050

//Test avec le filtre complémentaire => sur Arduino Uno!
//lecture des 2 valeurs: ~1420 us
//filtre complementaire: ~4us

//Test avec le filtre de Kalman (3 valeurs lues!)
//lecture des 2 valeurs: ~2380 us
//filtre kalman: ~350us

#include "gyropodeV3.h"
#include "Wire.h"
#include "MPU6050.h"
#include "I2Cdev.h"

//init MPU6050
void angleInit();
void initGyro();

//compute the angle (communication with MPU6050)…
//SLOW function
void angleAcquisition();

//get the last computed value
//angleAcquisition() should be called frequently.
//FAST function!
double getCompAngle();

double getRawAngle();

double getRawGyro();

double getGyroRate();

#include "angle.h"

double compAngle; // complementary angle between gyroscope and accelerometer
double a = 0.9;

double accAngle;
double gyroRate;
static unsigned long date; // for dt computation
double gyroYzero;          //offset gyro.

MPU6050 mpu;

bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference)
{ // check if robot is lying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++)
  {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return max - min < maxDifference;
}

bool calibrateGyro()
{
  int16_t gyroYbuffer[25];
  for (uint8_t i = 0; i < 25; i++)
  {
    gyroYbuffer[i] = mpu.getRotationY();
    delay(10);
  }
  if (!checkMinMax(gyroYbuffer, 25, 2000))
  {
    return 1;
  }
  for (uint8_t i = 0; i < 25; i++)
    gyroYzero += gyroYbuffer[i];
  gyroYzero /= 25;
  return 0;
}

void initGyro()
{
  //u TC4 => 1ms
  PM->APBCMASK.reg |= 1 << 12; // TC4
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN(0) |
                      GCLK_CLKCTRL_ID_TC4_TC5;
  TC4->COUNT8.PER.reg = 187 - 1; // 1ms/(16/3us)
  TC4->COUNT8.CTRLA.reg = 6 << 8 | 1 << 2 | 2;
  TC4->COUNT8.INTENSET.reg = 1;
  NVIC_EnableIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 1); //max priority
}

void angleInit()
{
  Wire.begin();
  mpu.initialize();
  while (calibrateGyro())
    ; // Run again if the robot is moved while calibrating
  date = millis();
}

void angleAcquisition()
{
  int16_t accX = mpu.getAccelerationX();
  int16_t accZ = mpu.getAccelerationZ();
  accAngle = atan2((double)accX, (double)accZ);                 // angle obtained using accelerometers
  
  int16_t gyroY = mpu.getRotationY();                           // get current gyro value
  gyroRate = -(((double)gyroY - gyroYzero) / 131) * DEG_TO_RAD; // compute gyro error in rad

  const float dt = (millis() - date) / 1000.;

  const float tmpComp = a * (compAngle + dt * gyroRate) + (1 - a) * accAngle;
  noInterrupts();
  compAngle = tmpComp;
  interrupts();
  date = millis();
}

double getCompAngle()
{
  volatile double tmp;
  noInterrupts();
  tmp = compAngle;
  interrupts();
  return tmp;
}

double getRawGyro()
{
  volatile double tmp;
  noInterrupts();
  tmp = gyroRate;
  interrupts();
  return tmp;
}

double getRawAngle()
{
  volatile double tmp;
  noInterrupts();
  tmp = accAngle;
  interrupts();
  return tmp;
}

double getGyroRate()
{
  volatile double tmp;
  noInterrupts();
  tmp = gyroRate;
  interrupts();
  return tmp;
}

#include "ble.h"
#include "encoder.h"
#include "angle.h"
#include "motors.h"

#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"

extern float batteryVoltage();

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void error(const __FlashStringHelper *err)
{
  Serial.println(err);
  while (1)
    ;
}

void BLEInit()
{
  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  // disable echo and debugging messages
  ble.echo(false);
  ble.verbose(false);

  // the robot will only work if there is an active connection
  while (!ble.isConnected())
  {
    delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    // Change Mode LED Activity
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // set to data mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

/*-------------------------------------------------------------------------------------
when the BLEhandler receives 'k' to update a gain, the next character is processed here
to determine which gain shall be updated. See gyropodeV3.h for details about the gains
----------------------------------------------------------------------------------------*/
void updateGain(char whichGain)
{
  switch (whichGain)
  {
  case 'p':
    Kp = ble.parseFloat();
    break;
  case 'v':
    Kv = ble.parseFloat();
    break;
  case 'b':
    Kbeta = ble.parseFloat();
    break;
  case 't':
    Ktheta = ble.parseFloat();
    break;
  case 'o':
    Komega = ble.parseFloat();
    break;
  }
}

/*--------------------------------------------------------------------------------------
print important robot data over bluetooth, useful when using smartphone
---------------------------------------------------------------------------------------*/
void printRobotData()
{
  // encoder values
  int32_t posLeft = getEncoder(LEFT);
  int32_t posRight = getEncoder(RIGHT);
  ble.print("Current Pos: ");
  ble.println((posLeft + posRight) / 2);
  ble.print("Encoders: ");
  ble.print(posLeft);
  ble.print(" - ");
  ble.println(posRight);

  // balance angle theta
  ble.print("theta: ");
  ble.println(getCompAngle() * RAD_TO_DEG);

  // gains
  ble.print("Kp: ");
  ble.print(Kp);
  ble.print(" - Kv: ");
  ble.print(Kv);
  ble.print(" - Kb: ");
  ble.println(Kbeta);
  ble.print("Kt: ");
  ble.print(Ktheta);
  ble.print(" - Ko: ");
  ble.println(Komega);

  // battery voltage
  ble.print("Bat: ");
  ble.println(batteryVoltage());
}

/*------------------------------------------------------------------------------------
very useful when using smarthphone, we can set new gain values, state references, run
or stop the controller and check more important data
--------------------------------------------------------------------------------------*/
void BLEhandler()
{
  int16_t pwm;
  char inChar = (char)ble.read();
  switch (inChar)
  {
  case 'p':
    // position reference
    pRef = ble.parseFloat();
    if (controllerActive)
      pRefTracking.setReference(pRef);
    ble.print("pos ref = ");
    ble.println(pRef);
    break;
  case 'b':
    // orientation beta reference in degrees
    betaRef = ble.parseFloat() * DEG_TO_RAD;
    ble.print("beta ref = ");
    ble.println(betaRef * RAD_TO_DEG);
    break;
  case 't':
    // balance theta reference in degrees
    thetaRef = ble.parseFloat() * DEG_TO_RAD;
    ble.print("theta ref = ");
    ble.println(thetaRef * RAD_TO_DEG);
    break;
  case 'k':
    // update gain of controller, the next character determines which gain
    inChar = (char)ble.read();
    updateGain(inChar);
    break;
  case 'i':
    printRobotData();
    break;
  case 'r':
    // starts the control
    resetEncoders();
    pRefTracking.init();
    pRefTracking.setReference(pRef);
    controllerActive = true;
    break;
  case 's':
    // stops controller and stop motors
    controllerActive = false;
    setMotor(RIGHT, 0);
    setMotor(LEFT, 0);
    break;
  case 'm':
    // drive each motor at a given pwm. separate values with comas ex:m255,255
    pwm = ble.parseInt();
    setMotor(LEFT, pwm);
    pwm = ble.parseInt();
    setMotor(RIGHT, pwm);
    break;
  case 'd':
    // starts to store data, rewrites previous data
    digitalWrite(LED_BUILTIN, LOW);
    indexData = 0;
    storeData = true;
    break;
  case 'u':
    // upload all stored data to pc
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(position[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(velocity[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(beta[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(theta[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(omega[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(outputLeft[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(outputRight[i]);
      Serial.print('\n');
    }
    for (uint16_t i = 0; i < NB_DATA_STORED; i++)
    {
      Serial.print(posReference[i]);
      Serial.print('\n');
    }
    break;
  default: // if there is still data in the buffer, read to empty
    ble.read();
    break;
  }
}
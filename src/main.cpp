#include "angle.h"
#include "ble.h"
#include "encoder.h"
#include "gyropodeV3.h"
#include "motors.h"

/*------------------------------------------------------------------------------------
see gyropodeV3.h for more info about the following parameters
-------------------------------------------------------------------------------------*/

// MOTOR PARAMETERRS
const double R = (6.5 / 2) / 100;

volatile bool controllerActive = false;
volatile bool storeData = false;
volatile double motionState;

// REFERENCE VALUES (THE VALUES WE WANT THE STATE TO CONVERGE TO)
volatile double pRef = 0;                     // ticks
volatile double vRef = 0;                     // m/s
volatile double betaRef = 0 * DEG_TO_RAD;     // deg
volatile double thetaRef = 11.5 * DEG_TO_RAD; // deg
volatile double omegaRef = 0;                 // rad/s

// CONTROLLER'S GAINS
// gains can change dynamically depending on the motion of the robot: {ACCELERATE, CONST_VELOCITY, DECELERATE, STOP}
volatile double Kp[4] = {-4, -0.3, -0.1, -0.3};      // -0.28 //-5
volatile double Kv[4] = {-20, -15, -10, -15};        // -77 //-150
volatile double Kip[4] = {0, 0, 0, 0};               // 0.001
volatile double Kbeta[4] = {25, 360, 25, 25};        // 250
volatile double Kib[4] = {0, 0, 0, 0};               //1.2
volatile double Ktheta[4] = {7000, 7000, 7500, 400}; //400
volatile double Komega[4] = {700, 700, 750, 50};     //50
volatile double Kit[4] = {0, 0, 0, 0};

// STORING OF DATA FOR LATER ANALYSIS

volatile uint16_t indexData = 0;
volatile float position[NB_DATA_STORED];
volatile float velocity[NB_DATA_STORED];
volatile float beta[NB_DATA_STORED];
volatile float theta[NB_DATA_STORED];
volatile float omega[NB_DATA_STORED];
volatile int16_t outputLeft[NB_DATA_STORED];
volatile int16_t outputRight[NB_DATA_STORED];
volatile float posReference[NB_DATA_STORED];
volatile float velReference[NB_DATA_STORED];
volatile float pvControlOutput[NB_DATA_STORED];
volatile float betaControlOutput[NB_DATA_STORED];
volatile float thetaControlOutput[NB_DATA_STORED];

int8_t sign(int32_t val)
{
  if (val < 0)
    return -1;
  else if (val > 0)
    return 1;
  else
    return 0;
}

/*-------------------------------------------------------------------------------------------
TC4 Handler. the main control is executed in this part. runs every 1ms
---------------------------------------------------------------------------------------------*/

referenceTracking pRefTracking(2, 1500, 0.001);

void TC4_Handler()
{
  static double pIntegral = 0;
  static double betaIntegral = 0;
  static double thetaIntegral = 0;

  if (controllerActive)
  {
    // use this counter i to do things with a period of more than 1 ms if needed
    static uint16_t i = 0;

    // meassure of encoder values for both wheels
    int32_t posRight = getEncoder(RIGHT);
    int32_t posLeft = getEncoder(LEFT);

    // For the speed:
    // we get from the encoders the sum of ticks of the last 32 ms (ticks/32ms)
    // to compute the actual speed in rad/s we consider the following:
    // divide by 32 -> ticks/ms
    // multiply by 1000 -> ticks/s
    // divide by 768 -> turns/s
    // multiply by 2pi -> rad/s
    int32_t leftSpeed, rightSpeed;
    getSpeed(leftSpeed, rightSpeed);
    double omegaRight = rightSpeed * 1000 * 2 * PI / (768 * 32);
    double omegaLeft = leftSpeed * 1000 * 2 * PI / (768 * 32);

    // TRACKING OF REFERENCES (for smoother control)
    static double tpRef = 0;
    static double tvRef = 0;
    // tracking reference and velocity !! getCurrentVelocity() must be always called after getCurrentReference() !!
    tpRef = pRefTracking.getCurrentRefence();
    //tvRef = pRefTracking.getCurrentVelocity() / NB_TICKS_PER_METER;
    // from reference tracking get the current motion state and choose the corresponding gains
    uint8_t chosenGain = pRefTracking.getState();

    // CONTROL OF POSITION AND VELOCITY OF THE ROBOT
    // we assume the robot cannot change beta (orientation) yet
    double pMes = (posLeft + posRight) / 2;
    double vMes = (omegaRight + omegaLeft) * R / 2;
    double posError = tpRef - pMes;
    pIntegral += posError;
    double P_pos = Kp[chosenGain] * posError;       // proportional action on p
    double D_pos = Kv[chosenGain] * (tvRef - vMes); // derivative action on p (velocity)
    double I_pos = Kip[chosenGain] * pIntegral;     // integral action on p
    // wind-up I_pos
    I_pos = constrain(I_pos, -255, 255);

    double pvControl = P_pos + D_pos + I_pos;

    // CONTROL OF THE ORIENTATION ANGLE OF THE ROBOT BETA
    double betaMes = (posRight - posLeft) * 2 * PI / NB_TICKS_PER_TURN;
    double betaError = betaRef - betaMes;
    betaIntegral += betaError;
    double P_beta = Kbeta[chosenGain] * betaError;  // proportional action
    double I_beta = Kib[chosenGain] * betaIntegral; // integral action
    // wind-up I_beta
    I_beta = constrain(I_beta, -255, 255);
    double betaControl = P_beta + I_beta;

    // CONTROL OF THE SELF BALANCE ANGLE THETA
    double thetaMes = getCompAngle();
    double omegaMes = getGyroRate();
    double thetaError = thetaRef - thetaMes;
    thetaIntegral += thetaError;
    double P_theta = Ktheta[chosenGain] * thetaError;            // proportional action on theta
    double D_theta = Komega[chosenGain] * (omegaRef - omegaMes); // derivative action on theta (omega)
    double I_theta = Kit[chosenGain] * thetaIntegral;            // integral action on theta
    // wind_up I_theta
    I_theta = constrain(I_theta, -255, 255);
    double thetaOmegacontrol = P_theta + D_theta + I_theta;

    // SET THE FINAL OUTPUT
    // betaControl is negative for one of the wheels cause the motors have to turn in different
    // directions for the robot to turn (duh!!)
    int16_t rightMotor = pvControl + thetaOmegacontrol + betaControl;
    int16_t leftMotor = pvControl + thetaOmegacontrol - betaControl;

    // finally we can set the motors outputs
    setMotor(RIGHT, rightMotor);
    setMotor(LEFT, leftMotor);

    // store data and time for later analysis. Data will be stored every DATA_SAMPLING_TIME ms
    // to get more memory efficiency and store more data
    if (i % DATA_SAMPLING_TIME == 0)
    {
      if (storeData == true)
      {
        if (indexData < NB_DATA_STORED)
        {
          position[indexData] = pMes;
          velocity[indexData] = vMes;
          beta[indexData] = betaMes;
          theta[indexData] = thetaMes;
          omega[indexData] = omegaMes;
          outputLeft[indexData] = leftMotor;
          outputRight[indexData] = rightMotor;
          posReference[indexData] = tpRef;
          velReference[indexData] = tvRef;
          pvControlOutput[indexData] = pvControl;
          betaControlOutput[indexData] = betaControl;
          thetaControlOutput[indexData] = thetaOmegacontrol;
          indexData++;
        }
        else
        {
          digitalWrite(LED_BUILTIN, HIGH); // led turns on when all data has been stored
          storeData = false;
        }
      }
    }
    i++;
  }
  else
  {
    pIntegral = 0;
    betaIntegral = 0;
    thetaIntegral = 0;
  }
  TC4->COUNT8.INTFLAG.reg = 1;
}

float batteryVoltage()
{
  return analogRead(A0) * (3.3 / 1024) * (15.1 / 10.0);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  BLEInit();
  initEncoders();
  initGyro();
  angleInit();
  initMotors();
}

void loop()
{
  angleAcquisition(); // this must be called very often
  if (ble.available())
    BLEhandler();
}
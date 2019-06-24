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
volatile double betaRef = 0;                  // deg
volatile double thetaRef = 11.5 * DEG_TO_RAD; // deg
volatile double omegaRef = 0;                 // rad/s

// CONTROLLER'S GAINS
// gains can change dynamically depending on the motion of the robot: {STOP and movement}
volatile double Kp[2] = {0.5, 0.08};      // {-1.21, -0.6, -1.21, -0.6}
volatile double Kv[2] = {-0.09, -0.11};  // {-100, -130, -100, -450}
volatile double Kip[2] = {0.00013, 0};   // {0, 0, 0, 0}
volatile double Kbeta[2] = {0.23, 0.23}; // {25, 25, 25, 27}
volatile double Kib[2] = {0, 0};         // {0 ,0 ,0 ,0}
volatile double Ktheta[2] = {600, 600};  // {1500, 1800, 2500, 550}
volatile double Komega[2] = {60, 60};    // {100, 150, 200, 60}
volatile double Kit[2] = {0, 0};         // {0, 0, 0, -0.0001}

// STORING OF DATA FOR LATER ANALYSIS

volatile uint16_t indexData = 0;
volatile float position[NB_DATA_STORED];
volatile float velocity[NB_DATA_STORED];
volatile float beta[NB_DATA_STORED];
volatile float theta[NB_DATA_STORED];
volatile float omega[NB_DATA_STORED];
// volatile int16_t outputLeft[NB_DATA_STORED];
// volatile int16_t outputRight[NB_DATA_STORED];
volatile float posReference[NB_DATA_STORED];
volatile float velReference[NB_DATA_STORED];
volatile float pvControlOutput[NB_DATA_STORED];
volatile float betaControlOutput[NB_DATA_STORED];
volatile float thetaControlOutput[NB_DATA_STORED];
volatile int16_t realOutputLeft[NB_DATA_STORED];
volatile int16_t realOutputRight[NB_DATA_STORED];

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

referenceTracking pRefTracking(2, MAX_SPEED_TICKS / 2, 0.001);

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
    int32_t leftSpeed, rightSpeed;
    getSpeed(leftSpeed, rightSpeed);
    double omegaRight = rightSpeed * 1000 / (32);
    double omegaLeft = leftSpeed * 1000 / (32);

    // TRACKING OF REFERENCES (for smoother control)
    static double tpRef = 0;
    static double tvRef = 0;
    // tracking reference and velocity !! getCurrentVelocity() must be always called after getCurrentReference() !!
    tpRef = pRefTracking.getCurrentRefence();
    tvRef = pRefTracking.getCurrentVelocity();
    // from reference tracking get the current motion state and choose the corresponding gain
    // we will use one gain for stop state and other for all the movements states
    static uint8_t lastChosenGain = 0;
    uint8_t chosenGain = 0;
    if (pRefTracking.getState() != STOP)
      chosenGain = 1;
    if (chosenGain != lastChosenGain)
    {
      pIntegral = 0;
      betaIntegral = 0;
      thetaIntegral = 0;
    }
    lastChosenGain = chosenGain;

    // CONTROL OF POSITION AND VELOCITY OF THE ROBOT
    // we assume the robot cannot change beta (orientation) yet
    double pMes = (posLeft + posRight) / 2;
    double vMes = (omegaRight + omegaLeft) / 2;
    double posError = tpRef - pMes;
    pIntegral += posError;
    double P_pos = Kp[chosenGain] * posError; // proportional action on p the output goes to the velocity ref
    double I_pos = Kip[chosenGain] * pIntegral;
    double D_pos = Kv[chosenGain] * (tvRef + P_pos + I_pos - vMes); // derivative action on p (velocity)
    // wind-up I_pos
    I_pos = constrain(I_pos, -MAX_PWM, MAX_PWM);

    double pvControl = D_pos;

    // CONTROL OF THE ORIENTATION ANGLE OF THE ROBOT BETA
    double betaMes = (posRight - posLeft) * RAD_TO_DEG * (2 * PI) / NB_TICKS_PER_TURN; // in deg
    double betaError = betaRef - betaMes;                                              // we put betaRef in ticks since we receive it in rad
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
    int16_t rightMotorOutput = pvControl + thetaOmegacontrol + betaControl;
    int16_t leftMotorOutput = pvControl + thetaOmegacontrol - betaControl;

    // map the output for each motor to the correct bounds of PWM_MIN and PWM_MAX
    int16_t rightMotorPWM = rightMotorOutput;
    int16_t leftMotorPWM = leftMotorOutput;
    if (abs(rightMotorOutput) > 0)
    {
      int16_t val = map(abs(rightMotorOutput), 0, 255, MIN_PWM, MAX_PWM);
      val = constrain(val, MIN_PWM, MAX_PWM);
      rightMotorPWM = sign(rightMotorOutput) * val;
    }
    if (abs(leftMotorOutput) > 0)
    {
      int16_t val = map(abs(leftMotorOutput), 0, 255, MIN_PWM, MAX_PWM);
      val = constrain(val, MIN_PWM, MAX_PWM);
      leftMotorPWM = sign(leftMotorOutput) * val;
    }

    // finally we can set the motors outputs
    setMotor(RIGHT, rightMotorPWM);
    setMotor(LEFT, leftMotorPWM);

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
          // outputLeft[indexData] = leftMotorOutput;
          // outputRight[indexData] = rightMotorOutput;
          posReference[indexData] = tpRef;
          velReference[indexData] = tvRef;
          pvControlOutput[indexData] = pvControl;
          betaControlOutput[indexData] = betaControl;
          thetaControlOutput[indexData] = thetaOmegacontrol;
          realOutputLeft[indexData] = leftMotorPWM;
          realOutputRight[indexData] = leftMotorPWM;
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
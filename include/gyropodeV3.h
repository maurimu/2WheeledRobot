#include "Arduino.h"
#include "referenceTracking.h"

#define NB_TICKS_PER_TURN 2548  // turn along the z axis (orientation of the robot.. NOT WHEELS !!!)
#define NB_TICKS_PER_METER 7522 // along x (direction of velocity)
#define MAX_LINEAR_SPEED 170    // max linear speed so that the beta control can have a better performance
#define NB_DATA_STORED 500      // total time that we will store for analysis
#define DATA_SAMPLING_TIME 10   // sampling time in milliseconds for data stored (not for control)

// max values
#define MAX_PINTEGRAL 2000
#define MAX_TINTEGRAL 1000
#define MAX_BINTEGRAL 1000

extern referenceTracking pRefTracking;

// ROBOT PARAMETERS
extern const double R; // wheels radius 6.5cm expressed in mm

// CONTROLLER PARAMETERS
/*--------------------------------------------------------------------------------------
We try to control the following states:
position of the robot: p
velocity of the robot along x axis: v
orientation (same direction as velocity): beta
angle of the inverted pendulum (balance): theta
rotation speed of the pendulum (dtheta/dt): omega
-----------------------------------------------------------------------------------------*/
extern volatile double pRef;     // desired p
extern volatile double vRef;     // desired v
extern volatile double betaRef;  // desired beta
extern volatile double thetaRef; // desired theta
extern volatile double omegaRef; // desired omega

/*----------------------------------------------------------------------------------------
The following gains will work alongside the error of all the states (ref - meassured)
the are set in main.cpp file
these are global variables since they are accesed by the bluetooth functions too !!

Kp: position proportional gain. Increases speed but also oscilations
Kv: position derivative gain (velocity). Decreases oscillations but also sensitivity
    to disturbances.
Kbeta: orientation proportional gain, same as Kp
Ktheta: balance angle proportional gain, same as Kp and Kbeta
Komega: balance angle derivative gain, same as Kv
-------------------------------------------------------------------------------------------*/
extern volatile double Kp[4];
extern volatile double Kv[4];
extern volatile double Kip[4];
extern volatile double Kbeta[4];
extern volatile double Kib[4];
extern volatile double Ktheta[4];
extern volatile double Komega[4];
extern volatile double Kit[4];

// The main switch... should be set to true if we want the contorller to work
// GLOBAL VARIABLE !
extern volatile bool controllerActive;

extern volatile bool storeData;
extern volatile uint16_t indexData;
extern volatile float position[NB_DATA_STORED];
extern volatile float velocity[NB_DATA_STORED];
extern volatile float beta[NB_DATA_STORED];
extern volatile float theta[NB_DATA_STORED];
extern volatile float omega[NB_DATA_STORED];
extern volatile int16_t outputLeft[NB_DATA_STORED];
extern volatile int16_t outputRight[NB_DATA_STORED];
extern volatile float posReference[NB_DATA_STORED];
extern volatile float velReference[NB_DATA_STORED];
extern volatile float pvControlOutput[NB_DATA_STORED];
extern volatile float betaControlOutput[NB_DATA_STORED];
extern volatile float thetaControlOutput[NB_DATA_STORED];

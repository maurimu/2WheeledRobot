#include "Arduino.h"

#define NB_TICKS_PER_TURN 2548  // turn along the z axis (orientation of the robot.. NOT WHEELS !!!)
#define NB_TICKS_PER_METER 7522 // along x (direction of velocity)
#define MAX_LINEAR_SPEED 170    // max linear speed so that the beta control can have a better performance
#define NB_DATA_STORED 1000     // total time that we will store for analysis
#define DATA_SAMPLING_TIME 3    // sampling time in milliseconds for data stored (not for control)

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
extern volatile double Kp;
extern volatile double Kv;
extern volatile double Kbeta;
extern volatile double Ktheta;
extern volatile double Komega;

// The main switch... should be set to true if we want the contorller to work
// GLOBAL VARIABLE !
extern volatile bool controllerActive;

extern uint16_t indexData;
extern float position[NB_DATA_STORED];
extern float velocity[NB_DATA_STORED];
extern float beta[NB_DATA_STORED];
extern float theta[NB_DATA_STORED];
extern float omega[NB_DATA_STORED];
extern int16_t outputLeft[NB_DATA_STORED];
extern int16_t outputRight[NB_DATA_STORED];

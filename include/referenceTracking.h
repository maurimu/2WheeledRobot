#ifndef __REFERENCE_TRACKING_H__
#define __REFERENCE_TRACKING_H__

#include "Arduino.h"

#define ACCELERATE 1
#define CONST_VELOCITY 2
#define DECELERATE 3
#define STOP 4

class referenceTracking
{
private:
    int16_t _state;
    int8_t _dir;
    float _reference;
    float _currentReference;
    float _currentVelocity = 0;

    // parameters:
    float _accelerationTime = 1; // in seconds
    float _maxVelocity = 300;    // in ticks/s
    float _sampleTime = 0.001;   // in seconds

public:
    referenceTracking();
    void init();
    void setAccelerationTime(float time) { _accelerationTime = time; };
    void setMaxVelocity(float velocity) { _maxVelocity = velocity; };
    void setSampleTime(float time) { _sampleTime = time; };
    void setReference(float ref);
    float getCurrentRefence();
};

#endif
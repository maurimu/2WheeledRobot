#ifndef __REFERENCE_TRACKING_H__
#define __REFERENCE_TRACKING_H__

#include "Arduino.h"

#define ACCELERATE 0
#define CONST_VELOCITY 1
#define DECELERATE 2
#define STOP 3

class referenceTracking
{
private:
    uint8_t _state;
    int8_t _dir;
    float _reference;
    float _currentReference;
    float _currentVelocity = 0;

    // parameters:
    float _accelerationTime; // in seconds
    float _maxVelocity;      // in ticks/s
    float _sampleTime;       // in seconds

public:
    referenceTracking(float accelerationTime, float maxVelocity, float sampleTime);
    void init();
    void setAccelerationTime(float time) { _accelerationTime = time; };
    void setMaxVelocity(float velocity) { _maxVelocity = velocity; };
    void setSampleTime(float time) { _sampleTime = time; };
    void setReference(float ref);
    float getCurrentRefence();
    float getCurrentVelocity() { return _currentVelocity; };
    uint8_t getState() { return _state; };
};

#endif
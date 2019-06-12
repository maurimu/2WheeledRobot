#include "referenceTracking.h"

referenceTracking::referenceTracking()
{
    init();
}

void referenceTracking::init()
{
    _state = STOP;
    _dir = 1;
    _reference = 0;
    _currentReference = 0;
    _currentVelocity = 0;
}

void referenceTracking::setReference(float ref)
{
    _state = ACCELERATE;
    if (ref > _currentReference)
        _dir = 1;
    else
        _dir = -1;
    _reference = ref;
}

float referenceTracking::getCurrentRefence()
{
    // the acceleration required to get to max velocity in the required time
    const float a = _maxVelocity / _accelerationTime;
    float decelerateReference;
    switch (_state)
    {
    case ACCELERATE:
        _currentVelocity += _dir * a * _sampleTime;
        _currentReference += _currentVelocity * _sampleTime;
        decelerateReference = _reference - _dir * _currentVelocity * _currentVelocity / (2 * a);
        // when max velocity is reached switch to constant velocity
        if (abs(_currentVelocity) > abs(_maxVelocity))
            _state = CONST_VELOCITY;
        // we may have to start decelerating before reaching the max velocity...
        if ((_dir && (_currentReference >= decelerateReference)) ||
            (_dir && (_currentReference <= decelerateReference)))
            _state = DECELERATE;
        break;
    case CONST_VELOCITY:
        _currentVelocity = _dir * _maxVelocity;
        _currentReference += _currentVelocity * _sampleTime;
        decelerateReference = _reference - _dir * _currentVelocity * _currentVelocity / (2 * a);
        if ((_dir && (_currentReference >= decelerateReference)) ||
            (_dir && (_currentReference <= decelerateReference)))
            _state = DECELERATE;
        break;
    case DECELERATE:
        _currentVelocity -= _dir * a * _sampleTime;
        _currentReference += _currentVelocity * _sampleTime;
        if (_currentVelocity * _dir <= 0)
        {
            _state = STOP;
            if (abs(_currentReference - _reference) > 10)
                setReference(_reference);
            else
                _currentReference = _reference;
        }
        break;
    case STOP:
        _currentVelocity = 0;
        break;
    default:
        _state = STOP;
    }
    return _currentReference;
}
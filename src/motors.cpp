#include "motors.h"

void initMotors()
{
    // this first call already configures the PMW
    analogWrite(PIN_RIGHT_MOTOR_PWM_FWD, 0);
    analogWrite(PIN_RIGHT_MOTOR_PWM_BCK, 0);

    // same for the other motor
    analogWrite(PIN_LEFT_MOTOR_PWM_FWD, 0);
    analogWrite(PIN_LEFT_MOTOR_PWM_BCK, 0);

    /*-----------------------------------------------------------------------
    reconfigure the frequency of timer TCC0 => 128x faster. For our motor
    driver the recommended pwm freq is between 20 and 100Khz
    -------------------------------------------------------------------------*/

    // stop the timer
    TCC0->CTRLA.bit.ENABLE = 0;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;
    // delete the old PRESCALER (256)
    TCC0->CTRLA.reg &= ~(TCC_CTRLA_PRESCALER_Msk << TCC_CTRLA_PRESCALER_Pos);
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;
    // and update it
    TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV2;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;
    // restart the timer
    TCC0->CTRLA.bit.ENABLE = 1;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;

    // we do the same thing for TC3 timer
    TC3->COUNT8.CTRLA.bit.ENABLE = 0;
    TC3->COUNT8.CTRLA.reg &= ~(TC_CTRLA_PRESCALER_Msk << TC_CTRLA_PRESCALER_Pos);
    TC3->COUNT8.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;
    TC3->COUNT8.CTRLA.bit.ENABLE = 1;
}

void setMotor(uint8_t id, int16_t speed)
{
    if (id > 1)
        return; // in case someone screws it up xD

    // make sure we stay in the interval [-MAX_PWM MAX_PWM]
    int16_t output;
    if (speed > MAX_PWM)
        output = MAX_PWM;
    else if (speed < -MAX_PWM)
        output = -MAX_PWM;
    else
        output = speed;

    // select which motor should be driven
    int pwm1, pwm2;
    if (id == RIGHT)
    {
        pwm1 = PIN_RIGHT_MOTOR_PWM_FWD;
        pwm2 = PIN_RIGHT_MOTOR_PWM_BCK;
    }
    else
    {
        pwm1 = PIN_LEFT_MOTOR_PWM_FWD;
        pwm2 = PIN_LEFT_MOTOR_PWM_BCK;
    }

    // select the direction of rotation
    if (speed > 0)
    { // forward
        analogWrite(pwm1, output);
        analogWrite(pwm2, 0);
    }
    else
    { // backwards
        analogWrite(pwm1, 0);
        analogWrite(pwm2, -output);
    }
}
#include "encoder.h"

volatile int32_t values[2] = {0, 0};

// LEFT ENCODER
void enc0AChange()
{
    if (digitalRead(PIN_ENC0_A) == HIGH)
    {
        if (digitalRead(PIN_ENC0_B) == HIGH)
            values[0]++;
        else
            values[0]--;
    }
    else
    {
        if (digitalRead(PIN_ENC0_B) == HIGH)
            values[0]--;
        else
            values[0]++;
    }
}

void enc0BChange()
{
    if (digitalRead(PIN_ENC0_B) == HIGH)
    {
        if (digitalRead(PIN_ENC0_A) == HIGH)
            values[0]--;
        else
            values[0]++;
    }
    else
    {
        if (digitalRead(PIN_ENC0_A) == HIGH)
            values[0]++;
        else
            values[0]--;
    }
}

// RIGHT ENCODER
void enc1AChange()
{
    if (digitalRead(PIN_ENC1_A) == HIGH)
    {
        if (digitalRead(PIN_ENC1_B) == HIGH)
            values[1]++;
        else
            values[1]--;
    }
    else
    {
        if (digitalRead(PIN_ENC1_B) == HIGH)
            values[1]--;
        else
            values[1]++;
    }
}

void enc1BChange()
{
    if (digitalRead(PIN_ENC1_B) == HIGH)
    {
        if (digitalRead(PIN_ENC1_A) == HIGH)
            values[1]--;
        else
            values[1]++;
    }
    else
    {
        if (digitalRead(PIN_ENC1_A) == HIGH)
            values[1]++;
        else
            values[1]--;
    }
}

#define NB_AVG_DATA 32 // number of meassures for the moving average

void initEncoders()
{
    resetEncoders();

    // left encoder
    pinMode(PIN_ENC0_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC0_A), enc0AChange, CHANGE);
    pinMode(PIN_ENC0_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC0_B), enc0BChange, CHANGE);

    //right encoder
    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), enc1AChange, CHANGE);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_B), enc1BChange, CHANGE);
}

int32_t deltaRight[NB_AVG_DATA];
int32_t sumRight;
uint8_t indexRight;
int32_t lastPosRight;

int32_t deltaLeft[NB_AVG_DATA];
int32_t sumLeft;
uint8_t indexLeft;
int32_t lastPosLeft;

int32_t getEncoder(uint8_t encoderId)
{
    if (encoderId > 1)
        return 0;
    int32_t result = values[encoderId];
    return result;
}

void resetEncoders()
{
    // position
    values[0] = 0;
    values[1] = 0;
    // average speed
    for (int i = 0; i < NB_AVG_DATA; i++)
    {
        deltaRight[i] = deltaLeft[i] = 0;
    }
    sumRight = sumLeft = 0;
    indexRight = indexLeft = 0;
    lastPosRight = lastPosLeft = 0;
}

// the speed is ticks of the encoder (dticks)
void getSpeed(int32_t &left, int32_t &right)
{
    // Right
    int32_t currentPosRight = values[RIGHT];
    int32_t newDeltaRight = currentPosRight - lastPosRight;
    sumRight -= deltaRight[indexRight];     // substract the old value
    sumRight += newDeltaRight;              // add the new one
    deltaRight[indexRight] = newDeltaRight; // and save it
    indexRight = (indexRight + 1) % NB_AVG_DATA;
    lastPosRight = currentPosRight;
    right = sumRight; // the sum of the last 32

    // Left ... same as avobe ;)
    int32_t currentPosLeft = values[LEFT];
    int32_t newDeltaLeft = currentPosLeft - lastPosLeft;
    sumLeft -= deltaLeft[indexLeft];
    sumLeft += newDeltaLeft;
    deltaLeft[indexLeft] = newDeltaLeft;
    indexLeft = (indexLeft + 1) % NB_AVG_DATA;
    lastPosLeft = currentPosLeft;
    left = sumLeft;
}
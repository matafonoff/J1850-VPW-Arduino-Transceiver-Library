#pragma once
#include <Arduino.h>

enum PIN_CHANGE {
    PIN_CHANGE_BOTH = 3,
    PIN_CHANGE_RISE = 1,
    PIN_CHANGE_FALL = 2
};

typedef void (*funcPtr)(uint8_t value);

void PCattachInterrupt(uint8_t pin, PIN_CHANGE mode, funcPtr pFunc);

void PCdetachInterrupt(uint8_t pin);

void PCpauseInterrupt(uint8_t pin);
void PCresumeInterrupt(uint8_t pin);

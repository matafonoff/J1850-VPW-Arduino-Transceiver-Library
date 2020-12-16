/*************************************************************************
**  AVR J1850VPW VPW Interface
**  by Stepan Matafonov
**
**  Released under Microsoft Public License
**
**  contact: matafonoff@gmail.com
**  homepage: xelb.ru
**************************************************************************/
#pragma once
#include <Arduino.h>

enum PIN_CHANGE {
    PIN_CHANGE_BOTH = 3,
    PIN_CHANGE_RISE = 1,
    PIN_CHANGE_FALL = 2
};

typedef void (*pCallbackFunction)(int state, void* pData);

void PCattachInterrupt(uint8_t pin, PIN_CHANGE mode, pCallbackFunction pFunc, void* pData);

void PCdetachInterrupt(uint8_t pin);

void PCpauseInterrupt(uint8_t pin);
void PCresumeInterrupt(uint8_t pin);

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
#include <functional>

enum PIN_CHANGE {
    PIN_CHANGE_BOTH = 3,
    PIN_CHANGE_RISE = 1,
    PIN_CHANGE_FALL = 2
};

void PCattachInterrupt(uint8_t pin, PIN_CHANGE mode, std::function<void(int)> pFunc);

void PCdetachInterrupt(uint8_t pin);

void PCpauseInterrupt(uint8_t pin);
void PCresumeInterrupt(uint8_t pin);

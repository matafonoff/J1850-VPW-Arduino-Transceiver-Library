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
#include "pins_arduino.h"
#include "interrupts.h"


enum PIN_MODES {
    PIN_MODE_INPUT = INPUT,
    PIN_MODE_INPUT_PULLUP = INPUT_PULLUP,
    PIN_MODE_OUTPUT = OUTPUT,
};

class Pin {
    volatile uint8_t* _reg;
    uint8_t _bit;
    uint8_t _pin;

    PIN_MODES _mode;
public:
    Pin();
    Pin(uint8_t pin, PIN_MODES mode);
    ~Pin();
public:
    void write(uint8_t val);
    uint8_t read();

    bool isEmpty() const;

    void attachInterrupt(PIN_CHANGE changeType, pCallbackFunction onPinChaged, void* pData);
    void detachInterrupt();

    void resumeInterrupts();
    void pauseInterrupts();
};



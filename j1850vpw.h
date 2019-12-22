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

#define J1850_ERR_BUS_IS_BUSY   0x81
#define J1850_ERR_BUS_ERROR     0x82

#define BS                      12

class J1850VPW {
public:
    static void setActiveLevel(uint8_t active);

    static void initReceiver(uint8_t pin);
    static void initTransmitter(uint8_t pin);

    static int8_t tryGetReceivedFrame(uint8_t *pBuff);
    static uint8_t send(uint8_t* pData, uint8_t size, uint32_t timeoutMs = -1);

    static void getFirstAndLast(uint8_t *pFirst, uint8_t *pLast);
};
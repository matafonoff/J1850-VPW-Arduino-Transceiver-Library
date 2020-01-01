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
#include "common.h"

enum J1850_ERRORS {
    J1850_OK                        = 0x00,
    J1850_ERR_BUS_IS_BUSY           = 0x81,
    J1850_ERR_BUS_ERROR             = 0x82,
    J1850_ERR_RECV_NOT_CONFIGURATED = 0x84,
    J1850_ERR_PULSE_TOO_SHORT       = 0x85,
    J1850_ERR_PULSE_OUTSIDE_FRAME   = 0x86,
    J1850_ERR_ARBITRATION_LOST      = 0x87,
    J1850_ERR_PULSE_TOO_LONG        = 0x88
};

enum J1850_Operations {
    J1850_Read,
    J1850_Write
};

typedef void (*onErrorHandler)(J1850_Operations op, J1850_ERRORS err);

class J1850VPW {
public:
    static void setActiveLevel(uint8_t active);

    static void init(uint8_t rxPin, uint8_t txPin);

    static int8_t tryGetReceivedFrame(uint8_t *pBuff, bool justValid = true);
    static uint8_t send(uint8_t* pData, uint8_t size, int16_t timeoutMs = -1);

    static void onError(onErrorHandler errHandler);
};
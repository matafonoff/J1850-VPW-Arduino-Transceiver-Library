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

#include "pins.h"
#include "storage.h"

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

class BitInfo
{
public:
    bool isActive;
    int length;
};

class J1850VPW {
private:
    uint8_t ACTIVE;
    uint8_t PASSIVE;
    int8_t RX;

    Pin __rxPin;
    Pin __txPin;

    ulong _lastChange;
    volatile bool _sofRead;
    volatile uint8_t _currState;
    volatile uint8_t _bit;
    volatile uint8_t _byte;
    uint8_t _buff[BS];
    uint8_t *msg_buf;

    Storage _storage = Storage();
    BitInfo bits[BS];

    volatile onErrorHandler __errHandler;

private:
    void onFrameRead();
    J1850_ERRORS handleErrorsInternal(J1850_Operations op, J1850_ERRORS err);
    void onRxChaged(uint8_t curr);

public:
    J1850VPW();

    void setActiveLevel(uint8_t active);

    void init(uint8_t rxPin, uint8_t txPin);

    int8_t tryGetReceivedFrame(uint8_t *pBuff, bool justValid = true);
    uint8_t send(uint8_t* pData, uint8_t size, int16_t timeoutMs = -1);

    void onError(onErrorHandler errHandler);
};
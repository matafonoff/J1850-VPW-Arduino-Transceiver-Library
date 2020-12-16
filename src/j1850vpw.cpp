/*************************************************************************
**  AVR J1850VPW VPW Interface
**  by Stepan Matafonov
**
**  Released under Microsoft Public License
**
**  contact: matafonoff@gmail.com
**  homepage: xelb.ru
**************************************************************************/
#include "j1850vpw.h"
#include <pins_arduino.h>
#include <stdlib.h>
#include "interrupts.h"

#define MAX(a, b) ((a) < (b) ? (b) : (a))

#define MAX_uS ((unsigned long )-1L)

#define RX_SOF_MIN (163)
#define RX_SOF_MAX (239)

#define RX_EOD_MIN (164) // minimum end of data time
#define RX_EOF_MIN (239) // minimum end of data time

#define RX_SHORT_MIN (34) // minimum short pulse time
#define RX_SHORT_MAX (96) // maximum short pulse time

#define RX_LONG_MIN (97)  // minimum long pulse time
#define RX_LONG_MAX (163) // maximum long pulse time

#define RX_EOD_MAX (239) // maximum end of data time

#define RX_PULSE_MAX (3000)

#define TX_SHORT (64) // Short pulse nominal time
#define TX_LONG (128) // Long pulse nominal time
#define TX_SOF (200)  // Start Of Frame nominal time
#define TX_EOD (200)  // End Of Data nominal time
#define TX_EOF (280)  // End Of Frame nominal time

#define IS_BETWEEN(x, min, max) (x >= min && x <= max)

uint8_t crc(uint8_t *msg_buf, int8_t nbytes)
{
    uint8_t crc_reg = 0xff, poly, byte_count, bit_count;
    uint8_t *byte_point;
    uint8_t bit_point;

    for (byte_count = 0, byte_point = msg_buf; byte_count < nbytes; ++byte_count, ++byte_point)
    {
        for (bit_count = 0, bit_point = 0x80; bit_count < 8; ++bit_count, bit_point >>= 1)
        {
            if (bit_point & *byte_point) // case for new bit = 1
            {
                if (crc_reg & 0x80)
                    poly = 1; // define the polynomial
                else
                    poly = 0x1c;
                crc_reg = ((crc_reg << 1) | 1) ^ poly;
            }
            else // case for new bit = 0
            {
                poly = 0;
                if (crc_reg & 0x80)
                    poly = 0x1d;
                crc_reg = (crc_reg << 1) ^ poly;
            }
        }
    }
    return ~crc_reg; // Return CRC
}


J1850_ERRORS J1850VPW::handleErrorsInternal(J1850_Operations op, J1850_ERRORS err)
{
    if (err != J1850_OK)
    {
        onErrorHandler errHandler = __errHandler;
        if (errHandler)
        {
            errHandler(op, err);
        }
    }

    return err;
}

bool J1850VPW::isReadonly() const {
    return __txPin.isEmpty();
}

void J1850VPW::onRxChaged(uint8_t curr)
{
    _currState = curr;
    curr = !curr;

    unsigned long  now = micros();
    unsigned long  diff;

    if (now < _lastChange)
    {
        // overflow occured
        diff = now + (MAX_uS - _lastChange);
    }
    else
    {
        diff = now - _lastChange;
    }

    _lastChange = now;

    if (diff < RX_SHORT_MIN)
    {
        // too short to be a valid pulse. Data error
        _sofRead = false;
        handleErrorsInternal(J1850_Read, J1850_ERR_PULSE_TOO_SHORT);
        return;
    }

    if (!_sofRead)
    {
        if (_sofRead = (curr == ACTIVE && IS_BETWEEN(diff, RX_SOF_MIN, RX_SOF_MAX)))
        {
            _byte = 0;
            _bit = 0;
            msg_buf = _buff;
            *msg_buf = 0;
        }
        else
        {
            handleErrorsInternal(J1850_Read, J1850_ERR_PULSE_OUTSIDE_FRAME);
        }
        return;
    }

    *msg_buf <<= 1;
    if (curr == PASSIVE)
    {
        if (diff > RX_EOF_MIN)
        {
            // data ended - copy package to buffer
            _sofRead = false;

            onFrameRead();
            return;
        }

        if (IS_BETWEEN(diff, RX_LONG_MIN, RX_LONG_MAX))
        {
            *msg_buf |= 1;
        }
    }
    else if (diff <= RX_SHORT_MAX)
    {
        *msg_buf |= 1;
    }

    _bit++;
    if (_bit == 8)
    {
        _byte++;
        msg_buf++;
        *msg_buf = 0;
        _bit = 0;
    }

    if (_byte == BS)
    {
        _sofRead = false;
        onFrameRead();
    }
}

J1850VPW::J1850VPW() 
: ACTIVE(LOW)
, PASSIVE(HIGH)
, RX(-1)
, __rxPin(Pin())
, __txPin(Pin())
, _lastChange(micros())
, _sofRead(false)
, _currState(ACTIVE)
, _bit(0)
, _byte(0)
, msg_buf(NULL)
, _storage(Storage())
, __errHandler(NULL)
{
    listenAll();
}

J1850VPW* J1850VPW::setActiveLevel(uint8_t active)
{
    if (active == LOW)
    {
        ACTIVE = LOW;
        PASSIVE = HIGH;
    }
    else
    {
        ACTIVE = HIGH;
        PASSIVE = LOW;
    }

    return this;
}

J1850VPW* J1850VPW::init(uint8_t rxPin, uint8_t txPin)
{
    init(rxPin);

    __txPin = Pin(txPin, PIN_MODE_OUTPUT);
    __txPin.write(PASSIVE);

    return this;
}

void J1850VPWFriend::__handleRnChange(int state, void* pData) {
    ((J1850VPW*)pData)->onRxChaged(state);
}

J1850VPW* J1850VPW::init(uint8_t rxPin) 
{
    __rxPin = Pin(rxPin, PIN_MODE_INPUT_PULLUP);

    _currState = __rxPin.read();

    __rxPin.attachInterrupt(PIN_CHANGE_BOTH, &J1850VPWFriend::__handleRnChange, this);

    return this;
}

// NB! Performance critical!!! Do not split
uint8_t J1850VPW::send(uint8_t *pData, uint8_t nbytes, int16_t timeoutMs /*= -1*/)
{
    if (isReadonly())
    {
        return handleErrorsInternal(J1850_Write, J1850_ERR_RECV_NOT_CONFIGURATED);
    }

    uint8_t result = J1850_OK;
    static uint8_t buff[BS];
    memcpy(buff, pData, nbytes);
    buff[nbytes] = crc(buff, nbytes);
    nbytes++;
    
    //         Serial.print("QQQ: ");
    // for (int q = 0; q < nbytes; q++) {
    //     uint8_t w = buff[q];
    //     if (w < 0x10) {
    //         Serial.print('0');
    //     }
    //     Serial.print(String(w, HEX));
    // }
    // Serial.println();
    
    __rxPin.pauseInterrupts();

    uint8_t *msg_buf = buff;
    unsigned long now;

    // wait for idle
    if (timeoutMs >= 0)
    {
        now = micros();
        unsigned long  start = now;
        timeoutMs *= 1000; // convert to microseconds
        while (micros() - now < TX_EOF)
        {
            if (__rxPin.read() == ACTIVE)
            {
                now = micros();
            }

            if (micros() - start > timeoutMs)
            {
                result = J1850_ERR_BUS_IS_BUSY;
                goto stop;
            }
        }
    }

    // SOF
    __txPin.write(ACTIVE);
    now = micros();
    while (micros() - now < TX_SOF)
        ;

    // send data
    do
    {
        uint8_t temp_byte = *msg_buf; // store byte temporary
        uint8_t nbits = 8;
        unsigned long  delay;
        while (nbits--) // send 8 bits
        {
            if (nbits & 1) // start allways with passive symbol
            {
                delay = (temp_byte & 0x80) ? TX_LONG : TX_SHORT; // send correct pulse lenght
                __txPin.write(PASSIVE);                          // set bus active
                now = micros();
                while (micros() - now < delay)
                {
                    if (__rxPin.read() == ACTIVE)
                    {
                        result = J1850_ERR_ARBITRATION_LOST;
                        goto stop;
                    }
                }
            }
            else // send active symbol
            {
                delay = (temp_byte & 0x80) ? TX_SHORT : TX_LONG; // send correct pulse lenght
                __txPin.write(ACTIVE);                           // set bus active
                now = micros();
                while (micros() - now < delay)
                    ;
            }

            temp_byte <<= 1; // next bit
        }                    // end nbits while loop
        ++msg_buf;           // next byte from buffer
    } while (--nbytes);      // end nbytes do loop

    // EOF
    __txPin.write(PASSIVE);
    now = micros();
    while (micros() - now < TX_SOF)
    {
        if (__rxPin.read() == ACTIVE)
        {
            result = J1850_ERR_ARBITRATION_LOST;
            goto stop;
        }
    }

stop:
    __rxPin.resumeInterrupts();
    return handleErrorsInternal(J1850_Write, result);
}

J1850VPW* J1850VPW::onError(onErrorHandler errHandler)
{
    __errHandler = errHandler;
    return this;
}

int8_t J1850VPW::tryGetReceivedFrame(uint8_t *pBuff, bool justValid /*= true*/)
{
    uint8_t size;

    while (true)
    {
        size = _storage.tryPopItem(pBuff);
        if (!size)
        {
            return 0;
        }

        if (!justValid || crc(pBuff, size - 1) == pBuff[size - 1])
        {
            break;
        }
    }

    return size;
}


void J1850VPW::onFrameRead()
{
    if (!IS_BETWEEN(_byte, 2, BS))
    {
        return;
    }

    uint8_t *pByte, bit;
    
    pByte = getBit(_buff[0], &bit);
    if (pByte && (*pByte & (1 << bit)) == 0) {
        _storage.push(_buff, _byte);
    }
}


J1850VPW* J1850VPW::listenAll() {
    memset(_ignoreList, 0, sizeof(_ignoreList));
    return this;
}

J1850VPW* J1850VPW::listen(uint8_t *ids) {
    uint8_t *pByte, bit;
    while (*ids) {
        pByte = getBit(*ids, &bit);

        if (pByte) {
            *pByte &= ~(1 << bit);
        }

        ids++;
    }
    return this;
}

J1850VPW* J1850VPW::ignoreAll() {
    memset(_ignoreList, 0xff, sizeof(_ignoreList));
    return this;
}

J1850VPW* J1850VPW::ignore(uint8_t *ids) {
    uint8_t *pByte, bit;
    while (*ids) {
        pByte = getBit(*ids, &bit);

        if (pByte) {
            *pByte |= 1 << bit;
        }

        ids++;
    }
    return this;
}

uint8_t* J1850VPW::getBit(uint8_t id, uint8_t *pBit) {
    if (!id) {
        *pBit = 0xff;
        return NULL;
    }

    *pBit = id % 8;
    return &(_ignoreList[id / 8]);
}
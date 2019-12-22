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

#define MAX_uS ((uint64_t)-1L)

#define RX_SOF_MIN (163)
#define RX_SOF_MAX (239)

#define RX_EOD_MIN (163) // minimum end of data time

#define RX_SHORT_MIN (34) // minimum short pulse time
#define RX_SHORT_MAX (96) // maximum short pulse time

#define RX_LONG_MIN (96)  // minimum long pulse time
#define RX_LONG_MAX (163) // maximum long pulse time

#define RX_EOD_MAX (239) // maximum end of data time

#define TX_SHORT (64) // Short pulse nominal time
#define TX_LONG (128) // Long pulse nominal time
#define TX_SOF (200)  // Start Of Frame nominal time
#define TX_EOD (200)  // End Of Data nominal time
#define TX_EOF (280)  // End Of Frame nominal time

#define IS_BETWEEN(x, min, max) (x >= min && x <= max)

static uint8_t ACTIVE = LOW;
static uint8_t PASSIVE = HIGH;
static uint8_t RX = -1;
static uint8_t TX = -1;

volatile uint8_t *port_to_pcmask[] = {
    &PCMSK0,
    &PCMSK1,
    &PCMSK2};

static int PCintMode;

typedef void (*funcPtr)(uint8_t value);
volatile static funcPtr PCintFunc = NULL;

volatile uint8_t PCintLast[3];
void onRxChaged(uint8_t curr);

void PCattachInterrupt(uint8_t pin, int mode)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *pcmask;

    // map pin to PCIR register
    if (port == NOT_A_PORT)
    {
        return;
    }
    else
    {
        port -= 2;
        pcmask = port_to_pcmask[port];
    }

    PCintMode = mode;
    PCintFunc = onRxChaged;
    // set the mask
    *pcmask |= bit;
    // enable the interrupt
    PCICR |= 0x01 << port;
}

void PCdetachInterrupt(uint8_t pin)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *pcmask;

    // map pin to PCIR register
    if (port == NOT_A_PORT)
    {
        return;
    }
    else
    {
        port -= 2;
        pcmask = port_to_pcmask[port];
    }

    // disable the mask.
    *pcmask &= ~bit;
    // if that's the last one, disable the interrupt.
    if (*pcmask == 0)
    {
        PCICR &= ~(0x01 << port);
    }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port)
{
    if (PCintFunc == NULL)
    {
        // No ISR handler attached. just skipping
        return;
    }
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;
    uint8_t currValue;

    // get the pin states for the indicated port.
    curr = *portInputRegister(port + 2);
    mask = curr ^ PCintLast[port];
    PCintLast[port] = curr;
    // mask is pins that have changed. screen out non pcint pins.
    if ((mask &= *port_to_pcmask[port]) == 0)
    {
        return;
    }

    // mask is pcint pins that have changed.
    for (uint8_t i = 0; i < 8; i++)
    {
        bit = 0x01 << i;
        if (bit & mask)
        {
            pin = port * 8 + i;
            // Trigger interrupt if mode is CHANGE, or if mode is RISING and
            // the bit is currently high, or if mode is FALLING and bit is low.
            currValue = curr & bit ? HIGH : LOW;
            if ((PCintMode == CHANGE || ((PCintMode == RISING) && currValue) || ((PCintMode == FALLING) && !currValue)))
            {
                PCintFunc(currValue);
            }
        }
    }
}

SIGNAL(PCINT0_vect)
{
    PCint(0);
}
SIGNAL(PCINT1_vect)
{
    PCint(1);
}
SIGNAL(PCINT2_vect)
{
    PCint(2);
}

uint64_t _lastChange = micros();
volatile bool _sofRead = false;
volatile uint8_t _currState;
volatile uint8_t _bit = 0;
volatile uint8_t _byte = 0;
uint8_t _buff[BS];
uint8_t *msg_buf;

#define STORAGE_SIZE 50

class StorageItem
{
public:
    uint8_t content[BS];
    uint8_t size;
};

static StorageItem _storage[STORAGE_SIZE] = {0};
volatile static int8_t _first = -1;
volatile static int8_t _last = -1;

void onFrameRead()
{
    if (!IS_BETWEEN(_byte, 0 , BS))
    {
        return;
    }

    _last++;
    if (_last >= STORAGE_SIZE)
    {
        _last = 0;
    }

    if (_first == -1)
    {
        _first = _last;
    }
    else if (_first == _last)
    {
        _first++;
        if (_first >= STORAGE_SIZE)
        {
            _first = 0;
        }
    }

    memcpy(_storage[_last].content, _buff, BS);
    _storage[_last].size = _byte;
}

void onRxChaged(uint8_t curr)
{
    _currState = curr;
    curr = !curr;

    uint64_t now = micros();
    uint64_t diff;

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
        return;
    }

    *msg_buf <<= 1;
    if (curr == PASSIVE)
    {
        if (diff > RX_EOD_MIN)
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
    else if (diff < RX_SHORT_MAX)
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

static void J1850VPW::setActiveLevel(uint8_t active)
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
}

static void J1850VPW::initReceiver(uint8_t pin)
{
    if (RX != -1)
    {
        PCdetachInterrupt(RX);
    }
    RX = pin;
    pinMode(RX, INPUT_PULLUP);
    PCattachInterrupt(RX, CHANGE);
}

uint8_t *_txReg = NULL;
uint8_t _txBit = 0;

inline bool sendPulse(uint8_t value, int16_t duration)
{
    uint64_t start = micros();

    if (value)
    {
        *_txReg &= ~_txBit;
    }
    else
    {
        *_txReg |= _txBit;
    }

    while (true)
    {
        if (value == PASSIVE && _currState != value)
        {
            // BUS ERROR
            return false;
        }
        uint64_t now = micros();

        if (now < start)
        {
            // overflow occured
            duration -= MAX_uS - start;
            start = 0;
        }

        if (now - start >= duration)
        {
            break;
        }
    }

    return true;
}

static void J1850VPW::initTransmitter(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    uint8_t timer = digitalPinToTimer(pin);
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PIN)
    {
        return;
    }

    // If the pin that support PWM output, we need to turn it off
    // before doing a digital write.
    // TODO turn pwm off
    // if (timer != NOT_ON_TIMER) {
    // 	turnOffPWM(timer);
    // }

    _txReg = portOutputRegister(port);
    _txBit = bit;
}

static uint8_t J1850VPW::send(uint8_t *msg_buf, uint8_t nbytes, uint32_t timeoutMs /*= -1*/)
{
    uint64_t now = millis();
    while (_sofRead || _currState != PASSIVE)
    {
        if (timeoutMs > 0 && millis() - now > timeoutMs)
        {
            return J1850_ERR_BUS_IS_BUSY;
        }
    }

    // SOF
    sendPulse(ACTIVE, TX_SOF);

    // send data
    do
    {
        uint8_t temp_byte = *msg_buf; // store byte temporary
        uint8_t nbits = 8;
        while (nbits--) // send 8 bits
        {
            uint8_t delay = (temp_byte & 0x80) ? TX_LONG : TX_SHORT; // send correct pulse lenght

            if (nbits & 1) // start allways with passive symbol
            {
                if (!sendPulse(PASSIVE, delay)) // set bus passive
                {
                    return J1850_ERR_BUS_ERROR; // error, bus collision!
                }
            }
            else // send active symbol
            {
                sendPulse(ACTIVE, delay); // set bus active
            }

            temp_byte <<= 1; // next bit
        }                    // end nbits while loop
        ++msg_buf;           // next byte from buffer
    } while (--nbytes);      // end nbytes do loop

    // EOF
    sendPulse(PASSIVE, TX_EOF);

    return 0;
}

    static void J1850VPW::getFirstAndLast(uint8_t *pFirst, uint8_t *pLast){
        *pFirst = _first;
        *pLast = _last;
    }

static int8_t J1850VPW::tryGetReceivedFrame(uint8_t *pBuff)
{
    if (_first == -1 || _last == -1)
    {
        return 0;
    }

    StorageItem *ptr = &_storage[_first];
    
    memcpy(pBuff, ptr->content, ptr->size);

    if (_first == _last)
    {
        _first = -1;
        _last = -1;
    }
    else
    {
        _first++;
        if (_first >= STORAGE_SIZE)
        {
            _first = 0;
        }
    }

   
    return ptr->size;
}

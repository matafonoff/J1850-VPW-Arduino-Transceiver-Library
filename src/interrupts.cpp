/*************************************************************************
**  AVR J1850VPW VPW Interface
**  by Stepan Matafonov
**
**  Released under Microsoft Public License
**
**  contact: matafonoff@gmail.com
**  homepage: xelb.ru
**************************************************************************/
#include "interrupts.h"

volatile uint8_t *port_to_pcmask[] = {
    &PCMSK0,
    &PCMSK1,
    &PCMSK2};

static PIN_CHANGE PCintMode[24];

volatile static pCallbackFunction PCintFunc[24] = {NULL};
volatile static void* PCintData[24] = {NULL};
volatile static bool PCintActive[24] = {NULL};

volatile static uint8_t PCintLast[3];

bool initPinInfo(uint8_t pin, uint8_t *pPort, uint8_t *pSlot)
{
    uint8_t port = digitalPinToPort(pin);
    // map pin to PCIR register
    if (port == NOT_A_PORT)
    {
        return false;
    }

    port -= 2;

    // -- Fix by Baziki. In the original sources it was a little bug, which cause analog ports to work incorrectly.
    if (port == 1)
    {
        *pSlot = port * 8 + (pin - 14);
    }
    else
    {
        *pSlot = port * 8 + (pin % 8);
    }
    // --Fix end

    *pPort = port;

    return true;
}

void PCattachInterrupt(uint8_t pin, PIN_CHANGE mode, pCallbackFunction pFunc, void* pData)
{
    uint8_t port;
    uint8_t slot;
    // map pin to PCIR register
    if (!initPinInfo(pin, &port, &slot))
    {
        return;
    }

    uint8_t bit = digitalPinToBitMask(pin);
    volatile uint8_t *pcmask = port_to_pcmask[port];

    PCintMode[slot] = mode;
    PCintFunc[slot] = pFunc;
    PCintData[slot] = pData;
    PCintActive[slot] = true;

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

void PCpauseInterrupt(uint8_t pin)
{
    uint8_t port;
    uint8_t slot;
    // map pin to PCIR register
    if (!initPinInfo(pin, &port, &slot))
    {
        return;
    }

    PCintActive[slot] = false;
}
void PCresumeInterrupt(uint8_t pin)
{
    uint8_t port;
    uint8_t slot;
    // map pin to PCIR register
    if (!initPinInfo(pin, &port, &slot))
    {
        return;
    }

    PCintActive[slot] = true;
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port)
{
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;

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
        if ((bit & mask))
        {
            pin = port * 8 + i;

            // Trigger interrupt if mode is CHANGE, or if mode is RISING and
            // the bit is currently high, or if mode is FALLING and bit is low.
            uint8_t currValue = curr & bit ? HIGH : LOW;
            if ((PCintFunc[pin] != NULL && PCintActive[pin]) &&
                (PCintMode[pin] == PIN_CHANGE_BOTH || ((PCintMode[pin] == PIN_CHANGE_RISE) && currValue) || ((PCintMode[pin] == PIN_CHANGE_FALL) && !currValue)))
            {
                PCintFunc[pin](currValue, PCintData[pin]);
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
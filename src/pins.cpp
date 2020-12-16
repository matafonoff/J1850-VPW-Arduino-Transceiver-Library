/*************************************************************************
**  AVR J1850VPW VPW Interface
**  by Stepan Matafonov
**
**  Released under Microsoft Public License
**
**  contact: matafonoff@gmail.com
**  homepage: xelb.ru
**************************************************************************/
#include "pins.h"
#include "wiring_private.h"

static void turnOffPWM(uint8_t timer)
{
    switch (timer)
    {
#if defined(TCCR1A) && defined(COM1A1)
    case TIMER1A:
        cbi(TCCR1A, COM1A1);
        break;
#endif
#if defined(TCCR1A) && defined(COM1B1)
    case TIMER1B:
        cbi(TCCR1A, COM1B1);
        break;
#endif
#if defined(TCCR1A) && defined(COM1C1)
    case TIMER1C:
        cbi(TCCR1A, COM1C1);
        break;
#endif

#if defined(TCCR2) && defined(COM21)
    case TIMER2:
        cbi(TCCR2, COM21);
        break;
#endif

#if defined(TCCR0A) && defined(COM0A1)
    case TIMER0A:
        cbi(TCCR0A, COM0A1);
        break;
#endif

#if defined(TCCR0A) && defined(COM0B1)
    case TIMER0B:
        cbi(TCCR0A, COM0B1);
        break;
#endif
#if defined(TCCR2A) && defined(COM2A1)
    case TIMER2A:
        cbi(TCCR2A, COM2A1);
        break;
#endif
#if defined(TCCR2A) && defined(COM2B1)
    case TIMER2B:
        cbi(TCCR2A, COM2B1);
        break;
#endif

#if defined(TCCR3A) && defined(COM3A1)
    case TIMER3A:
        cbi(TCCR3A, COM3A1);
        break;
#endif
#if defined(TCCR3A) && defined(COM3B1)
    case TIMER3B:
        cbi(TCCR3A, COM3B1);
        break;
#endif
#if defined(TCCR3A) && defined(COM3C1)
    case TIMER3C:
        cbi(TCCR3A, COM3C1);
        break;
#endif

#if defined(TCCR4A) && defined(COM4A1)
    case TIMER4A:
        cbi(TCCR4A, COM4A1);
        break;
#endif
#if defined(TCCR4A) && defined(COM4B1)
    case TIMER4B:
        cbi(TCCR4A, COM4B1);
        break;
#endif
#if defined(TCCR4A) && defined(COM4C1)
    case TIMER4C:
        cbi(TCCR4A, COM4C1);
        break;
#endif
#if defined(TCCR4C) && defined(COM4D1)
    case TIMER4D:
        cbi(TCCR4C, COM4D1);
        break;
#endif

#if defined(TCCR5A)
    case TIMER5A:
        cbi(TCCR5A, COM5A1);
        break;
    case TIMER5B:
        cbi(TCCR5A, COM5B1);
        break;
    case TIMER5C:
        cbi(TCCR5A, COM5C1);
        break;
#endif
    }
}

bool Pin::isEmpty() const
{
    return this->_reg == NULL;
}

void Pin::write(uint8_t val)
{
    uint8_t oldSREG = SREG;
    cli();

    if (val == LOW)
    {
        *this->_reg &= ~this->_bit;
    }
    else
    {
        *this->_reg |= this->_bit;
    }

    SREG = oldSREG;
}

uint8_t Pin::read()
{
    if (*this->_reg & this->_bit)
        return HIGH;
    return LOW;
}

void Pin::attachInterrupt(PIN_CHANGE changeType, pCallbackFunction onPinChaged, void* pData)
{
    PCattachInterrupt(this->_pin, changeType, onPinChaged, pData);
}
void Pin::detachInterrupt()
{
    PCdetachInterrupt(this->_pin);
}

void Pin::resumeInterrupts()
{
    PCresumeInterrupt(this->_pin);
}

void Pin::pauseInterrupts()
{
    PCpauseInterrupt(this->_pin);
}

Pin ::~Pin()
{
    this->detachInterrupt();
}

Pin::Pin()
{
    this->_reg = NULL;
    this->_bit = -1;
    this->_mode = 0;
    this->_pin = -1;
}

Pin::Pin(uint8_t pin, PIN_MODES mode)
{
    uint8_t timer = digitalPinToTimer(pin);
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PIN)
    {
        return;
    }

    // If the pin that support PWM output, we need to turn it off
    // before getting a digital reading.
    if (timer != NOT_ON_TIMER)
    {
        turnOffPWM(timer);
    }

    // JWS: can I let the optimizer do this?
    volatile uint8_t *reg;
    volatile uint8_t *out;
    reg = portModeRegister(port);
    out = portOutputRegister(port);
    if (mode == PIN_MODE_INPUT)
    {
        uint8_t oldSREG = SREG;
        cli();
        *reg &= ~bit;
        *out &= ~bit;
        SREG = oldSREG;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        uint8_t oldSREG = SREG;
        cli();
        *reg &= ~bit;
        *out |= bit;
        SREG = oldSREG;
    }
    else
    {
        uint8_t oldSREG = SREG;
        cli();
        *reg |= bit;
        SREG = oldSREG;
    }

    if (mode == PIN_MODE_OUTPUT)
    {
        reg = portOutputRegister(port);
    }
    else
    {
        reg = portInputRegister(port);
    }

    this->_reg = reg;
    this->_bit = bit;
    this->_mode = mode;
    this->_pin = pin;
}

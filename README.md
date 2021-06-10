# J1850-VPW-Arduino-Transceiver-Library
Arduino library which allow to communicate on J1850-VPW mode.
It works with wire. Should work with radio and laser transmissions as well.
You just have to choose tx and rx pins.
It doesn't have either a header nor a parity check, just CRC check. 
All validations could be implemented at a higher protocol layer.

This library has been tested with wire connection between two Arduino and J1850 vpw bus of Chrysler Pacifica vehicle. 

## Connection
Use following schematics to connect Arduino to real J1850VPW bus of your vehicle:
![schematics](img/schematics.jpg)

Connections:
* IN -> pin of Arduino that is configured as RX pin for J1850VPW
* OUT -> pin of Arduino that is configured as TX pin for J1850VPW
* BUS -> J1850VPW bus (ex. vehicle)

## Code example
~~~~

#include "j1850vpw.h"

#define TX 9
#define RX 10

void handleError(J1850_Operations op, J1850_ERRORS err);

J1850VPW vpw;

void setup()
{
    Serial.begin(115200);           // start serial port
    vpw.onError(handleError); // listen for errors
    vpw.init(RX, TX);         // init transceiver
}

void loop()
{
    static uint8_t buff[BS];                   // buffer for read message
    static uint8_t sendBuff[2] = {0x01, 0x02}; // message to send, CRC will be read and appended to frame on the fly

    uint8_t dataSize; // size of message

    // read all messages with valid CRC from cache
    while (dataSize = vpw.tryGetReceivedFrame(buff))
    {
        String s;

        // convert to hex string
        uint8_t *pData = buff;
        for (int i = 0; i < dataSize; i++)
        {
            if (i > 0)
            {
                s += " ";
            }

            if (buff[i] < 0x10)
            {
                s += '0';
            }

            s += String(pData[i], HEX);
        }

        Serial.println(s);
    }

    // write simple message to bus
    vpw.send(sendBuff, sizeof(sendBuff));

    delay(1000);
}


void handleError(J1850_Operations op, J1850_ERRORS err)
{
    if (err == J1850_OK)
    {
        // skip non errors if any
        return;
    }

    String s = op == J1850_Read ? "READ " : "WRITE ";
    switch (err)
    {
    case J1850_ERR_BUS_IS_BUSY:
        Serial.println(s + "J1850_ERR_BUS_IS_BUSY");
        break;
    case J1850_ERR_BUS_ERROR:
        Serial.println(s + "J1850_ERR_BUS_ERROR");
        break;
    case J1850_ERR_RECV_NOT_CONFIGURATED:
        Serial.println(s + "J1850_ERR_RECV_NOT_CONFIGURATED");
        break;
    case J1850_ERR_PULSE_TOO_SHORT:
        Serial.println(s + "J1850_ERR_PULSE_TOO_SHORT");
        break;
    case J1850_ERR_PULSE_OUTSIDE_FRAME:
        Serial.println(s + "J1850_ERR_PULSE_OUTSIDE_FRAME");
        break;
    case J1850_ERR_ARBITRATION_LOST:
        Serial.println(s + "J1850_ERR_ARBITRATION_LOST");
        break;
    case J1850_ERR_PULSE_TOO_LONG:
        Serial.println(s + "J1850_ERR_PULSE_TOO_LONG");
        break;
    case J1850_ERR_IFR_RX_NOT_SUPPORTED:
        Serial.println(s + "J1850_ERR_IFR_RX_NOT_SUPPORTED");
        break;
    default:
        // unknown error
        Serial.println(s + "ERR: " + String(err, HEX));
        break;
    }
}
~~~~

#### Based on:
1. Some code from wiring sources of arduino to implement fast digital IO.

## NB!
This software was developed to be used with Chrysler vehicles in a mind. This means that the current protocol implementation misses some features like (IFR).

If you have any ideas of how to implement features you need - feel free to make a pull request. In this case you will be mentioned as a library developer and this library will be developed much faster.

## P.S.
This library is rewritten by me to SM chips and I'm working on that code primerially. And yes, I'm not working with Arduino for few monthes already.
This means I have little time to support Arduino implementation. So if you have any code updates - feel free to use pull requests.

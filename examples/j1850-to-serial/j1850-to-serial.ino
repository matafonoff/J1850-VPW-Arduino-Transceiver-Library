
#include "j1850vpw.h"

#define TX 9
#define RX 10

void handleError(J1850_Operations op, J1850_ERRORS err);
J1850VPW vpw;

void setup()
{
    Serial.begin(9600);           // start serial port
    Serial.println("OK");
    delay(300);
    if (Serial.available() > 0) {
      switch(Serial.read()) {
        case '0': Serial.begin(115200); break;
        case '1': Serial.begin(56800); break;
        case '2': Serial.begin(38400);
        default:  Serial.begin(19200); break;
      }
    }
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

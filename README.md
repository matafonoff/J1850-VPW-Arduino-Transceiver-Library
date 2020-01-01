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
* IN -> RX pin of Arduino
* OUT -> TX pin of Arduino
* BUS -> J1850VPW bus (ex. vehicle)

## Code example

#### Based on:
1. Some code from wiring sources of arduino to implement fast digital IO.
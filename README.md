# J1850-VPW-Arduino-Transceiver-Library
Arduino library which allow to communicate on J1850-VPW mode.
It works with wire. Should work with radio and laser transmissions as well.
You just have to choose your bit-rate (lower is better), tx and rx pins.
It doesn't have either a header nor a parity check. These functions could be implemented at a higher protocol layer.

Required libraries:
1. EnableInterrupt  (Tested with version 0.9.5, Download: https://github.com/GreyGnome/EnableInterrupt)

This library has been tested with:
1. Wire connection between two Arduino

## 1. Wire connection between two Arduino (DEFAULT)
Arduino Nano (TRX)  |  Arduino Nano (TRX)  |    Notes  
-------------|-------------|------------
9 | 10 | Intelligent signal Tx
10 | 9 | Intelligent signal Rx
GND | GND | Common GND 

#### Remember: 
In order to keep the transmission medium free (ether) after transmission, it is recommended to use the "setTxRxModePin" method (see example sketch "Advanced_TX"). 
The "mode_pin" is connected to the radio module and is used to choose the transmission directions (to select transmission or reception, see datasheet of RTX-MID-868 module https://goo.gl/UEv8ii).

#### Based on:
1. J1850-Arduino-Transceiver-Library library (https://github.com/VittorioEsposito/J1850-Arduino-Transceiver-Library)
2. Code from https://www.mictronics.de/projects/j1850-vpw-interface/ . You can also get wiring schematics from this page as well.
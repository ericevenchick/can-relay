# CAN Relay #
Controls relays over CAN using a PIC18F26K80 MCU.

### CAN Specs ###
Select CAN ID and baud rate when building (main.c)
CAN Frame structure:
* Byte 1: Command
  * 0 = read status
  * 1 = set status
* Byte 2: (LSB) RA0 - RA3, RA5, RB0, RB1 (MSB)
* Byte 3: (LSB) RB5 - RB7, RC0 - RC4 (MSB)


# AT Keyboard as HID USB Keyboard

This is a code for Arduino Pro Micro board
to connect an old pure serial AT keyboard
to a modern PC via USB.

ATmega32U4 chip is used as HID Keyboard device.

It's based on LUFA library.

Connect devices:

| AT Keyboard   | Arduino Pro Micro | ATmega32U4 |
|---------------|-------------------|------------|
| GND           | GND               | GND        |
| VCC           | VCC               | VCC        |
| CLOCK         | D3                | PD0 (INT0) |
| DATA          | D2                | PD1        |

Build:

```bash
git clone https://github.com/abcminiuser/lufa.git 
make clean
make
# double link (2 times, fast): RST + GND on the board
# to enter the bootloader mode for 8 seconds
make flash
```

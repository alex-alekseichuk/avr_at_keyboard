#include <avr/io.h>
#include "leds_on_off.h"

void on1() { PORTB &= ~(1 << PB0); }
void on2() { PORTD &= ~(1 << PD5); }
void off1() { PORTB |= (1 << PB0); }
void off2() { PORTD |= (1 << PD5); }
void init_on_off() {
    DDRB |= (1 << PB0);   // set pin as output
    DDRD |= (1 << PD5);   // set pin as output
}

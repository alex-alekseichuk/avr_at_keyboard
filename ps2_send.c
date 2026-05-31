#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ps2_send.h"

#define CLK_PORT PORTD
#define CLK_DDR  DDRD
#define CLK_PIN  PD2

#define DAT_PORT PORTD
#define DAT_DDR  DDRD
#define DAT_PIN  PD3

static inline void clk_low(void)
{
    CLK_DDR |= (1 << CLK_PIN);
    CLK_PORT &= ~(1 << CLK_PIN);
}

static inline void clk_release(void)
{
    CLK_DDR &= ~(1 << CLK_PIN);
}

static inline void dat_low(void)
{
    DAT_DDR |= (1 << DAT_PIN);
    DAT_PORT &= ~(1 << DAT_PIN);
}

static inline void dat_release(void)
{
    DAT_DDR &= ~(1 << DAT_PIN);
}

static inline uint8_t clk_read(void)
{
    return PIND & (1 << CLK_PIN);
}

static void wait_clk_low(void)
{
    while (clk_read());
}

static void wait_clk_high(void)
{
    while (!clk_read());
}

void ps2_send(uint8_t data)
{
    uint8_t parity = 1;

    cli();

    // disable INT0 receive ISR
    EIMSK &= ~(1 << INT0);

    // -------------------------
    // Request to send
    // -------------------------

    clk_low();
    _delay_us(150);

    dat_low();

    clk_release();

    // keyboard starts clocking
    wait_clk_low();
    wait_clk_high();

    // -------------------------
    // send 8 bits
    // -------------------------

    for (uint8_t i = 0; i < 8; i++) {

        if (data & 1) {
            dat_release();
            parity ^= 1;
        } else {
            dat_low();
        }

        wait_clk_low();
        wait_clk_high();

        data >>= 1;
    }

    // -------------------------
    // parity bit
    // -------------------------

    if (parity)
        dat_release();
    else
        dat_low();

    wait_clk_low();
    wait_clk_high();

    // -------------------------
    // stop bit
    // -------------------------

    dat_release();

    wait_clk_low();
    wait_clk_high();

    // -------------------------
    // ACK bit
    // keyboard pulls DATA low
    // -------------------------

    wait_clk_low();
    wait_clk_high();

    // re-enable receiver
    EIFR |= (1 << INTF0);
    EIMSK |= (1 << INT0);

    sei();
}

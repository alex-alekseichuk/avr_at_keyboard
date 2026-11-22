#include <stdbool.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ps2_send.h"

#define CLK_PORT PORTD
#define CLK_DDR  DDRD
#define CLK_PIN  PD0

#define DAT_PORT PORTD
#define DAT_DDR  DDRD
#define DAT_PIN  PD1

/*
 * Max number of loop iterations while waiting for a clock edge before giving
 * up. A PS/2 bit takes at most ~100 us, so ~50 ms of waiting is safely above
 * any legitimate bit time while still bounding a hang.
 */
#define CLK_WAIT_MAX 100000

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

static bool wait_clk_low(void)
{
    for (uint16_t i = 0; i < CLK_WAIT_MAX; i++)
        if (!clk_read())
            return true;
    return false;
}

static bool wait_clk_high(void)
{
    for (uint16_t i = 0; i < CLK_WAIT_MAX; i++)
        if (clk_read())
            return true;
    return false;
}

/* Give up on a send: release both lines and re-arm the receiver. */
static void send_abort(void)
{
    dat_release();
    clk_release();
    EIFR |= (1 << INTF0);
    EIMSK |= (1 << INT0);
    sei();
}

void ps2_send(uint8_t data)
{
    /* Even parity: parity bit is 1 iff the data has an odd number of 1 bits. */
    uint8_t parity = 0;

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
    if (!wait_clk_low() || !wait_clk_high()) {
        send_abort();
        return;
    }

    // -------------------------
    // send 8 bits, MSB first
    // -------------------------

    for (int i = 7; i >= 0; i--) {

        if (data & (1 << i)) {
            dat_release();
            parity ^= 1;
        } else {
            dat_low();
        }

        if (!wait_clk_low() || !wait_clk_high()) {
            send_abort();
            return;
        }
    }

    // -------------------------
    // parity bit
    // -------------------------

    if (parity)
        dat_release();
    else
        dat_low();

    if (!wait_clk_low() || !wait_clk_high()) {
        send_abort();
        return;
    }

    // -------------------------
    // stop bit
    // -------------------------

    dat_release();

    if (!wait_clk_low() || !wait_clk_high()) {
        send_abort();
        return;
    }

    // -------------------------
    // ACK bit
    // keyboard pulls DATA low
    // -------------------------

    if (!wait_clk_low() || !wait_clk_high()) {
        send_abort();
        return;
    }

    // re-enable receiver
    EIFR |= (1 << INTF0);
    EIMSK |= (1 << INT0);

    sei();
}

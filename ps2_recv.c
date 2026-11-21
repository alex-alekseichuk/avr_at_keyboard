#include <stdint.h>

#include <avr/interrupt.h>

#include "recv_queue.h"

static volatile uint8_t ps2_bitcount = 0;
static volatile uint8_t ps2_byte = 0;

ISR(INT0_vect)
{
    uint8_t bit = (PIND & (1 << 1)) ? 1 : 0;

    if (ps2_bitcount == 0)
    {
        ps2_bitcount = 1;
        ps2_byte = 0;
        return;
    }

    if (ps2_bitcount >= 1 && ps2_bitcount <= 8)
    {
        ps2_byte >>= 1;
        if (bit)
            ps2_byte |= 0x80;
        ps2_bitcount++;
        return;
    }

    if (ps2_bitcount == 9)
    {
        ps2_bitcount++;
        return;
    }

    if (ps2_bitcount == 10)
    {
        ps2_bitcount = 0;
        push_to_recv_queue(ps2_byte);
    }
}


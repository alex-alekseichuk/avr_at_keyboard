#include <avr/interrupt.h>

#include "recv_queue.h"

#define PS2_QUEUE_SIZE 16
#define PS2_QUEUE_MASK (PS2_QUEUE_SIZE - 1)

static volatile uint8_t ps2_queue[PS2_QUEUE_SIZE];
static volatile uint8_t ps2_head = 0;
static volatile uint8_t ps2_tail = 0;
static volatile bool _is_ps2_queue_overflow = false;

bool recv_dequeue(uint8_t *byte)
{
    bool has_got = false;

    cli();
    if (ps2_tail != ps2_head)
    {
        *byte = ps2_queue[ps2_tail];
        ps2_tail = (ps2_tail + 1) & PS2_QUEUE_MASK;
        has_got = true;
    }
    sei();

    return has_got;
}

bool push_to_recv_queue(uint8_t ps2_byte) {
    uint8_t next = (ps2_head + 1) & PS2_QUEUE_MASK;
    if (next != ps2_tail)
    {
        ps2_queue[ps2_head] = ps2_byte;
        ps2_head = next;
    } else {
        _is_ps2_queue_overflow = true;
    }
}

void reset_ps2_queue_overflow() {
    _is_ps2_queue_overflow = false;
}

bool is_ps2_queue_overflow() {
    return _is_ps2_queue_overflow;
}

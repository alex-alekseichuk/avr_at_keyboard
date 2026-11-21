#include <stdbool.h>
#include <stdint.h>

#include "ps2_send.h"
#include "send_queue.h"

#define SEND_QUEUE_SIZE 16
#define SEND_QUEUE_MASK (SEND_QUEUE_SIZE - 1)
static volatile uint8_t send_queue[SEND_QUEUE_SIZE];
static volatile uint8_t send_head = 0;
static volatile uint8_t send_tail = 0;
static volatile bool may_send_next_byte = true;

void push_to_send_queue(uint8_t send_byte) {
    uint8_t next = (send_head + 1) & SEND_QUEUE_MASK;
    if (next != send_tail) {
        send_queue[send_head] = send_byte;
        send_head = next;
    }
}

void allow_send_next_byte() {
    may_send_next_byte = true;
}

void process_send_queue() {
    if (may_send_next_byte && send_tail != send_head) {
        uint8_t byte = send_queue[send_tail];
        send_tail = (send_tail + 1) & SEND_QUEUE_MASK;
        may_send_next_byte = false;
        ps2_send(byte);
    }
}

#pragma once
#include <stdbool.h>
#include <stdint.h>

bool recv_dequeue(uint8_t *byte);
bool push_to_recv_queue(uint8_t ps2_byte);
void reset_ps2_queue_overflow();
bool is_ps2_queue_overflow();

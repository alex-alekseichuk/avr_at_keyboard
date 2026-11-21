#pragma once

#include <stdint.h>

void push_to_send_queue(uint8_t send_byte);
void allow_send_next_byte();
void process_send_queue();

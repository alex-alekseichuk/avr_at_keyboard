#pragma once

#include <stdint.h>

/**
 * Send a byte to AT keyboard.
 * @param data
 */
void ps2_send(uint8_t data);

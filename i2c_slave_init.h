#pragma once

#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>

void i2c2__slave_init(i2c_e i2c, uint8_t slave_address_to_respond_to);

/*
bool i2c_slave_callback__read_memory(uint8_t memory_index, uint8_t *memory);

bool i2c_slave_callback__write_memory(uint8_t memory_index, uint8_t memory_value);
*/
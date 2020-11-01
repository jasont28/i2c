#include "i2c_slave_init.h"
#include "lpc40xx.h"
#include <stdio.h>
void i2c2__slave_init(i2c_e i2c, uint8_t slave_address_to_respond_to) {
  LPC_I2C_TypeDef *LPC_I2C = (i2c == I2C__0) ? LPC_I2C0 : ((i2c == I2C__1) ? LPC_I2C1 : LPC_I2C2);
  LPC_I2C->ADR0 = slave_address_to_respond_to; // Write address
  LPC_I2C->CONSET = 0x44;                      // Write 0x44 to enable slave mode
}

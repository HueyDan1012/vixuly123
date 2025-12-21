#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#define SCL_CLOCK  400000L

// Fucntions prototypes
void i2c_init(void);
uint8_t i2c_start(uint8_t address);
void i2c_stop(void);
uint8_t i2c_write(uint8_t data);

uint8_t i2c_read_ack(void); // Read with ACK 
uint8_t i2c_read_nack(void); // Read with NACK 

#endif // I2C_H
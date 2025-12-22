/*
 * i2c.c
 * Implementation of TWI (I2C) for ATmega328P
 */

#include <avr/io.h>
#include "i2c.h"

void I2C_Init(void)
{
    // 1. Set Prescaler to 1 (TWPS0 = 0, TWPS1 = 0)
    // TWSR status register bits 0 and 1 control prescaler
    TWSR = 0x00;

    // 2. Calculate and Set Bit Rate Register (TWBR)
    // Formula: TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2
    TWBR = (uint8_t)(((F_CPU / SCL_CLOCK) - 16) / 2);

    // 3. Enable TWI Module
    // TWEN: TWI Enable bit in TWCR (Control Register)
    TWCR = (1 << TWEN);
}

void I2C_Start(void)
{
    // Send Start Condition
    // TWINT: Clear interrupt flag to start operation
    // TWSTA: Start condition bit
    // TWEN: Enable TWI
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    // Wait for TWINT flag to be set (operation complete)
    while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop(void)
{
    // Send Stop Condition
    // TWSTO: Stop condition bit
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    
    // Note: We don't wait for TWINT after Stop, the hardware handles it automatically.
}

void I2C_Write(uint8_t data)
{
    // Load data into TWI Data Register
    TWDR = data;

    // Clear TWINT to start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for transmission to complete
    while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_Read_Ack(void)
{
    // Read byte and send ACK (Acknowledge)
    // TWEA: TWI Enable Acknowledge bit
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

    // Wait for data to be received
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}

uint8_t I2C_Read_Nack(void)
{
    // Read byte and send NACK (No Acknowledge)
    // Used for the last byte of a read sequence
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for data to be received
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}
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
volatile uint8_t i2c_error_flag = 0;
void I2C_WaitForComplete(void) {
    uint16_t timeout_counter = 0;
    while (!(TWCR & (1 << TWINT))) 
    {
        timeout_counter++;
        if (timeout_counter >= I2C_TIMEOUT) 
        {
            // --- XỬ LÝ KHI BỊ TREO ---
            i2c_error_flag = 1; // Đánh dấu lỗi
            
            // Cưỡng chế Reset lại module TWI để thoát trạng thái treo
            TWCR = 0; 
            TWCR = (1 << TWEN); 
            return; // Thoát ngay
        }
    }
}
void I2C_Start(void)
{
    // Send Start Condition
    // TWINT: Clear interrupt flag to start operation
    // TWSTA: Start condition bit
    // TWEN: Enable TWI
    i2c_error_flag = 0;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    // Wait for TWINT flag to be set (operation complete)
    I2C_WaitForComplete();
}

void I2C_Stop(void)
{
    // Send Stop Condition
    // TWSTO: Stop condition bit
    if (i2c_error_flag) return;
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    
    // Note: We don't wait for TWINT after Stop, the hardware handles it automatically.
}

void I2C_Write(uint8_t data)
{
    // Load data into TWI Data Register
    if (i2c_error_flag) return;
    TWDR = data;

    // Clear TWINT to start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for transmission to complete
    I2C_WaitForComplete();
}

uint8_t I2C_Read_Ack(void)
{
    if (i2c_error_flag) return 0; // Trả về 0 nếu đang lỗi

    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    
    I2C_WaitForComplete();
    
    if (i2c_error_flag) return 0; // Trả về 0 nếu timeout
    return TWDR;
}

uint8_t I2C_Read_Nack(void)
{
    if (i2c_error_flag) return 0;

    TWCR = (1 << TWINT) | (1 << TWEN);
    
    I2C_WaitForComplete();
    
    if (i2c_error_flag) return 0;
    return TWDR;
}
uint8_t I2C_IsError(void) {
    return i2c_error_flag;
}
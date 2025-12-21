#include "i2c.h"
#include <avr/io.h>
#include <util/twi.h>
void i2c_init(void) {
    // Set SCL frequency
    TWSR = 0; // Prescaler value = 1
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;

    // Enable TWI
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(uint8_t address) {
    // Gửi START
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // --- THÊM TIMEOUT ---
    uint16_t timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 1; // Báo lỗi 1 (Timeout)
    }
    // --------------------

    // Gửi Địa chỉ
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN);

    // --- THÊM TIMEOUT ---
    timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 1; // Báo lỗi 1
    }
    // --------------------
    
    return 0; // Thành công
}

void i2c_stop(void) {
    // Send STOP condition
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t i2c_write(uint8_t data) {
    // Load data into TWDR register
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    return 0; // Return 0 on success
}

uint8_t i2c_read_ack(void) {
    // Read data with ACK
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}

uint8_t i2c_read_nack(void) {
    // Read data with NACK
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}

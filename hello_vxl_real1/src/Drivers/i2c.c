#include "i2c.h"
#include <avr/io.h>
#include <util/twi.h>

void i2c_init(void) {
    // Set SCL frequency = 400kHz
    TWSR = 0; // Prescaler value = 1
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    // Enable TWI
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(uint8_t address) {
    // Gửi START
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // Timeout cho START
    uint16_t timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 1; // Lỗi Timeout
    }

    // Gửi Địa chỉ
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Timeout cho Gửi Địa chỉ
    timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 1; // Lỗi Timeout
    }
    
    return 0; // Thành công
}

void i2c_stop(void) {
    // Gửi STOP
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // [QUAN TRỌNG] Thêm Timeout cho Write
    uint16_t timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 1; // Lỗi Timeout
    }

    return 0; 
}

uint8_t i2c_read_ack(void) {
    // Đọc với ACK
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    
    // [QUAN TRỌNG] Thêm Timeout cho Read ACK
    uint16_t timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 0xFF; // Trả về rác nếu lỗi
    }

    return TWDR;
}

uint8_t i2c_read_nack(void) {
    // Đọc với NACK (byte cuối)
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // [QUAN TRỌNG] Thêm Timeout cho Read NACK
    uint16_t timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (--timeout == 0) return 0xFF; // Trả về rác nếu lỗi
    }

    return TWDR;
}
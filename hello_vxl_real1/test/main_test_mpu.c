/*
 * main_test_mpu.c
 * Kiểm tra đọc dữ liệu MPU6050 qua I2C và gửi về UART
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>

#include "i2c.h"
#include "mpu6050.h"

// --- UART SETUP (Để gửi dữ liệu lên máy tính) ---
void UART_Init(void) {
    // 9600 baud @ 16MHz -> UBRR = 103
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Bật Tx, Rx
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void UART_Tx(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Chờ bộ đệm trống
    UDR0 = data;
}

void UART_Print(char *str) {
    while (*str) UART_Tx(*str++);
}

// --- BIẾN TOÀN CỤC ---
MPU6050_t sensor;
char buffer[100]; // Buffer để chứa chuỗi in ra

int main(void) {
    // 1. Khởi tạo
    UART_Init();
    I2C_Init();
    MPU6050_Init();
    
    // Bật đèn LED PB5 (D13) để báo nguồn
    DDRB |= (1 << PB5); 

    UART_Print("--- MPU6050 TEST START ---\r\n");
    _delay_ms(1000);

    while (1) {
        // 2. Đọc dữ liệu
        MPU6050_Read_All(&sensor);

        // 3. Tính góc thử nghiệm (Accel Angle)
        float angle = atan2(sensor.Ay, sensor.Az) * 57.296;

        // 4. In ra Serial
        // Lưu ý: sprintf %f có thể bị tắt mặc định trên AVR-GCC để tiết kiệm nhớ.
        // Ta ép kiểu về int để in cho an toàn và nhẹ.
        sprintf(buffer, "GyroX_Raw: %d | Acc_Angle: %d\r\n", 
                (int)sensor.Gyro_X_RAW, 
                (int)angle);
        
        UART_Print(buffer);

        // Nháy đèn mỗi lần đọc để biết code đang chạy
        PORTB ^= (1 << PB5);

        _delay_ms(100); // Đọc 10 lần/giây để dễ nhìn
    }
}
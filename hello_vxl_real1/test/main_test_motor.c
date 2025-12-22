/*
 * main_test_motor.c
 * Kiểm tra động cơ DC, chiều quay và PWM Timer1
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "dc_motor.h"

// Dùng đèn LED D13 (PB5) để báo trạng thái
#define LED_PIN PB5

int main(void) {
    // 1. Khởi tạo
    Motor_Init();
    Motor_Stop();
    
    // Cấu hình LED
    DDRB |= (1 << LED_PIN);

    // Chờ 2 giây trước khi chạy để bạn kịp đặt robot xuống đất (hoặc kê cao bánh xe)
    // Nháy đèn báo hiệu chuẩn bị
    for(int i=0; i<10; i++) {
        PORTB ^= (1 << LED_PIN);
        _delay_ms(200);
    }
    PORTB &= ~(1 << LED_PIN); // Tắt đèn

    while (1) {
        
        // --- TEST 1: CHẠY TIẾN (FORWARD) ---
        PORTB |= (1 << LED_PIN); // Đèn sáng khi tiến
        
        // Tăng tốc từ 0 đến Max
        for (int16_t speed = 0; speed <= 255; speed += 5) {
            Motor_L_Control(speed);
            Motor_R_Control(speed);
            _delay_ms(50); // Tăng tốc từ từ
        }
        
        _delay_ms(1000); // Chạy Max tốc độ trong 1s

        // Giảm tốc về 0
        for (int16_t speed = 255; speed >= 0; speed -= 5) {
            Motor_L_Control(speed);
            Motor_R_Control(speed);
            _delay_ms(50);
        }

        Motor_Stop();
        PORTB &= ~(1 << LED_PIN); // Tắt đèn
        _delay_ms(1000); // Nghỉ 1s

        // --- TEST 2: CHẠY LÙI (REVERSE) ---
        // Đèn nháy nhanh khi lùi
        
        // Tăng tốc lùi (speed âm)
        for (int16_t speed = 0; speed <= 255; speed += 5) {
            Motor_L_Control(-speed); // Dấu trừ để lùi
            Motor_R_Control(-speed);
            
            PORTB ^= (1 << LED_PIN); // Nháy đèn
            _delay_ms(50);
        }

        _delay_ms(1000);

        // Giảm tốc lùi về 0
        for (int16_t speed = 255; speed >= 0; speed -= 5) {
            Motor_L_Control(-speed);
            Motor_R_Control(-speed);
            
            PORTB ^= (1 << LED_PIN);
            _delay_ms(50);
        }

        Motor_Stop();
        PORTB &= ~(1 << LED_PIN);
        _delay_ms(2000); 
    }
}
/*
 * dc_motor.c
 * Implementation using Timer1 (16-bit) in 8-bit Fast PWM Mode
 */

#include "dc_motor.h"
#include <stdlib.h> // for abs()

void Motor_Init(void)
{
    // 1. Cấu hình chân Output
    // Chân hướng (Direction) PD4, PD5, PD6, PD7
    MOTOR_DIR_DDR |= (1 << M1_IN1_PIN) | (1 << M1_IN2_PIN) | 
                     (1 << M2_IN1_PIN) | (1 << M2_IN2_PIN);

    // Chân PWM PB1, PB2
    M1_PWM_DDR |= (1 << M1_PWM_PIN);
    M2_PWM_DDR |= (1 << M2_PWM_PIN);

    // 2. Cấu hình Timer1 (16-bit) chế độ Fast PWM 8-bit
    // Chúng ta muốn đếm từ 0 đến 255 (0x00FF) để tương thích với output PID 8-bit
    
    // TCCR1A:
    // COM1A1 = 1, COM1B1 = 1: Non-inverting PWM (Xóa khi đạt so sánh, set khi về đáy)
    // WGM10  = 1: Chọn chế độ Fast PWM 8-bit (Mode 5) - Cùng với WGM12 bên dưới
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);

    // TCCR1B:
    // WGM12 = 1: Tiếp tục chọn Mode 5 (Fast PWM 8-bit)
    // CS11  = 1: Prescaler = 8
    // Tần số PWM = 16MHz / (8 * 256) = 7812.5 Hz -> Rất êm và phản ứng nhanh
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Khởi đầu tắt motor
    OCR1A = 0; // Duty Cycle M1
    OCR1B = 0; // Duty Cycle M2
}

void Motor_L_Control(int16_t speed) {
    // 1. BÙ VÙNG CHẾT (Logic quan trọng nhất)
    if (speed > 0) {
        speed += MIN_PWM; // Ví dụ: PID tính ra 5 -> Motor nhận 55 -> QUAY ĐƯỢC
    } else if (speed < 0) {
        speed -= MIN_PWM; // Ví dụ: PID tính ra -5 -> Motor nhận -55
    }

    // 2. ĐIỀU KHIỂN CHIỀU VÀ GIỚI HẠN
    if (speed > 0) {
        if (speed > 255) speed = 255;
        PORTD |= (1 << PD4);
        PORTD &= ~(1 << PD5);
        OCR1A = (uint8_t)speed;
    } 
    else if (speed < 0) {
        speed = -speed;
        if (speed > 255) speed = 255;
        PORTD &= ~(1 << PD4);
        PORTD |= (1 << PD5);
        OCR1A = (uint8_t)speed;
    } 
    else {
        // Speed = 0 -> Dừng hẳn
        PORTD &= ~(1 << PD4);
        PORTD &= ~(1 << PD5);
        OCR1A = 0;
    }
}

void Motor_R_Control(int16_t speed) {
    // Tương tự cho motor phải
    if (speed > 0) {
        speed += MIN_PWM;
    } else if (speed < 0) {
        speed -= MIN_PWM;
    }

    if (speed > 0) {
        if (speed > 255) speed = 255;
        PORTD |= (1 << PD6);
        PORTD &= ~(1 << PD7);
        OCR1B = (uint8_t)speed;
    } 
    else if (speed < 0) {
        speed = -speed;
        if (speed > 255) speed = 255;
        PORTD &= ~(1 << PD6);
        PORTD |= (1 << PD7);
        OCR1B = (uint8_t)speed;
    } 
    else {
        PORTD &= ~(1 << PD6);
        PORTD &= ~(1 << PD7);
        OCR1B = 0;
    }
}

void Motor_Stop(void) {
    PORTD &= ~(1 << PD4); PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6); PORTD &= ~(1 << PD7);
    OCR1A = 0; OCR1B = 0;
}
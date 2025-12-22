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

void Motor_L_Control(int16_t speed)
{
    // Motor Left dùng PD4, PD5
    if (speed > 0) 
    {
        // Tiến
        MOTOR_DIR_PORT |= (1 << M1_IN1_PIN);
        MOTOR_DIR_PORT &= ~(1 << M1_IN2_PIN);
    } 
    else if (speed < 0) 
    {
        // Lùi
        MOTOR_DIR_PORT &= ~(1 << M1_IN1_PIN);
        MOTOR_DIR_PORT |= (1 << M1_IN2_PIN);
        speed = -speed; // Lấy trị tuyệt đối
    } 
    else 
    {
        // Dừng thả trôi (Coasting) hoặc Phanh (Braking)
        // Ở đây dùng Phanh (Low-Low) để robot đứng vững hơn
        MOTOR_DIR_PORT &= ~(1 << M1_IN1_PIN);
        MOTOR_DIR_PORT &= ~(1 << M1_IN2_PIN);
        OCR1A = 0;
        return;
    }

    // Giới hạn tốc độ
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < MIN_PWM) speed = 0; // Cắt bỏ vùng chết

    // Ghi vào thanh ghi Timer1 kênh A (PB1)
    OCR1A = (uint16_t)speed;
}

void Motor_R_Control(int16_t speed)
{
    // Motor Right dùng PD6, PD7
    if (speed > 0) 
    {
        MOTOR_DIR_PORT |= (1 << M2_IN1_PIN);
        MOTOR_DIR_PORT &= ~(1 << M2_IN2_PIN);
    } 
    else if (speed < 0) 
    {
        MOTOR_DIR_PORT &= ~(1 << M2_IN1_PIN);
        MOTOR_DIR_PORT |= (1 << M2_IN2_PIN);
        speed = -speed;
    } 
    else 
    {
        MOTOR_DIR_PORT &= ~(1 << M2_IN1_PIN);
        MOTOR_DIR_PORT &= ~(1 << M2_IN2_PIN);
        OCR1B = 0;
        return;
    }

    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < MIN_PWM) speed = 0;

    // Ghi vào thanh ghi Timer1 kênh B (PB2)
    OCR1B = (uint16_t)speed;
}

void Motor_Stop(void)
{
    // Tắt hết các chân Direction
    MOTOR_DIR_PORT &= ~((1 << M1_IN1_PIN) | (1 << M1_IN2_PIN) | 
                        (1 << M2_IN1_PIN) | (1 << M2_IN2_PIN));
    OCR1A = 0;
    OCR1B = 0;
}
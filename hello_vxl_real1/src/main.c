/*
 * SELF-BALANCING ROBOT FIRMWARE (SPEED OPTIMIZED)
 * -----------------------------------------------
 * Loop Rate: 100Hz (10ms) via Timer0
 * UART Rate: 115200 Baud (Fast Debugging)
 * Deadzone Compensation: Enabled in dc_motor.c
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- INCLUDE DRIVERS ---
#include "i2c.h"
#include "mpu6050.h"
#include "dc_motor.h"
#include "filter.h"
#include "pid.h"

// =============================================================
//                CẤU HÌNH (TUNING PARAMETERS)
// =============================================================

// 1. OFFSET GYRO
#define GYRO_OFFSET_HARDCODE 10

// 2. PID PARAMETERS (Đã nhân 1000)
// Kp=20.0, Ki=0.08, Kd=1.2
// Nếu robot vẫn yếu -> Tăng Kp lên 25000, 30000
#define KP_VAL          8000L 
#define KI_VAL          20L    
#define KD_VAL          1800L  

// 3. GÓC MỤC TIÊU
#define TARGET_ANGLE    0      

// =============================================================
//                     BIẾN TOÀN CỤC
// =============================================================
volatile uint8_t sys_tick_flag = 0; 
volatile uint8_t timer_counter = 0; 

MPU6050_t mpu;
CompFilter_t angle_filter;
PID_Config_t balance_pid;

// =============================================================
//                     DRIVER SETUP
// =============================================================

// --- UART DEBUG (HIGH SPEED: 115200) ---
void UART_Init(void) {
    // Để đạt 115200 baud chuẩn ở 16MHz, cần bật chế độ nhân đôi tốc độ (U2X0)
    // Công thức: UBRR = (F_CPU / (8 * Baud)) - 1
    // UBRR = (16000000 / (8 * 115200)) - 1 = 16.36 -> Lấy 16
    
    UBRR0H = 0; 
    UBRR0L = 16; 
    
    UCSR0A |= (1 << U2X0); // Bật Double Speed 
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_Tx(char data) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = data; }
void UART_Print(char *str) { while (*str) UART_Tx(*str++); }

// --- HÀM IN DEBUG ---
void Print_Debug(int32_t angle, int16_t pwm) {
    static uint8_t count = 0;
    count++;
    // In mỗi 100ms (10 ticks)
    if (count >= 10) { 
        count = 0;
        char buf[64];
        sprintf(buf, "A: %ld | P: %d\r\n", angle/1000, pwm);
        UART_Print(buf);
        
        // Nháy đèn báo hiệu
        PORTB ^= (1 << PB5); 
    }
}

// --- TIMER0 INIT (100Hz Loop) ---
void Timer0_Init(void) {
    TCCR0A = 0;
    // Prescaler 64: 1 Tick = 4us. Tràn sau 256 tick = 1.024ms
    TCCR0B = (1 << CS01) | (1 << CS00); 
    TCNT0 = 0;
    TIMSK0 |= (1 << TOIE0);
}

// --- TIMER0 ISR ---
ISR(TIMER0_OVF_vect) {
    timer_counter++;
    // 1.024ms * 10 ~= 10.24ms -> ~100Hz
    if (timer_counter >= 10) {
        timer_counter = 0;
        sys_tick_flag = 1; // Đánh thức Loop
    }
}

// =============================================================
//                        MAIN PROGRAM
// =============================================================
int main(void) {
    // 1. Setup Hardware
    DDRB |= (1 << PB5); // LED Debug
    UART_Init();      // 115200 baud
    
    MPU6050_Init();
    Motor_Init();
    Motor_Stop();

    // 2. Setup Middleware
    CompFilter_Init(&angle_filter, 980); // Tin Gyro 98%
    PID_Init(&balance_pid, KP_VAL, KI_VAL, KD_VAL, 255); // Max PWM 255

    // Bật đèn báo hiệu đã Init xong
    PORTB |= (1 << PB5);
    _delay_ms(500);
    PORTB &= ~(1 << PB5);

    // 3. Start System
    Timer0_Init();
    sei(); // Bật ngắt toàn cục

    while (1) {
        // Chờ Timer báo hiệu (đúng mỗi 10ms)
        if (sys_tick_flag == 1) {
            sys_tick_flag = 0; // Xóa cờ ngay

            // --- A. ĐỌC CẢM BIẾN ---
            MPU6050_Read_All(&mpu);

            // --- B. XỬ LÝ SỐ LIỆU ---
            float acc_angle_f = atan2((float)mpu.Accel_Y_RAW, (float)mpu.Accel_Z_RAW) * 57.296f;
            int32_t acc_angle_int = (int32_t)(acc_angle_f * 1000);

            int32_t gyro_val = mpu.Gyro_X_RAW - GYRO_OFFSET_HARDCODE;
            int32_t gyro_rate_int = (gyro_val * 15267L) / 1000;

            // --- C. BỘ LỌC (FILTER) ---
            int32_t current_angle = CompFilter_Update(&angle_filter, acc_angle_int, gyro_rate_int, 10);

            // --- D. PID COMPUTE ---
            int16_t pid_output = PID_Compute(&balance_pid, current_angle, TARGET_ANGLE * 1000);

            // --- E. MOTOR CONTROL ---
            // Đã có Deadzone Compensation trong file dc_motor.c
            Motor_L_Control(pid_output);
            Motor_R_Control(pid_output);

            // --- F. DEBUG ---
            Print_Debug(current_angle, pid_output);
        }
    }
}
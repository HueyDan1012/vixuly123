// /*
//  * Self-Balancing Robot Firmware
//  * Method: Fixed Sampling Loop using Timer2 Interrupt
//  * Fix: Throttled UART output for stability
//  */


// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <util/delay.h>
// #include <math.h>
// #include <stdlib.h>
// #include <stdio.h>

// #include "i2c.h"
// #include "mpu6050.h"
// #include "filter.h"
// #include "pid.h"
// #include "dc_motor.h"

// // --- CONFIGURATION ---
// // Timer2: 100Hz (10ms)
// #define PERIOD 100
// #define LED PB5
// // --- GLOBAL VARIABLES ---
// volatile uint8_t sys_tick_flag = 0; // Cờ ngắt
// volatile uint8_t timer_counter = 0;
// char buffer[64];                    // Giảm buffer xuống 64 cho nhẹ RAM
// MPU6050_t mpu_data;
// CompFilter_t angle_filter;


// // Biến đếm để giảm tần suất in Serial
// uint8_t print_counter = 0; 

// // --- UART SETUP ---
// void UART_Init(void) {
//     UBRR0H = 0; UBRR0L = 103; // 9600 baud
//     UCSR0B = (1 << RXEN0) | (1 << TXEN0);
//     UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
// }
// void UART_Tx(char data) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = data; }
// void UART_Print(char *str) { while (*str) UART_Tx(*str++); }



// void Timer0_Init() {
//     TCCR0A = 0x00;
//     TCCR0B = ( 1<< CS01 ) | ( 1 << CS00 );

//     TCNT0 = 6;
//     TIMSK0 |= 1 << TOIE0; // CHO PHEP NGAT TRAN TIMER0
   
    
// }
// // --- TIMER0 ISR ---
// ISR(TIMER0_OVF_vect) 
// {   
//     TCNT0 = 6;
  
//     if (++timer_counter == PERIOD) {
//         // Reset biến đếm
//          PORTB ^= 1 << LED;
//          timer_counter=0;
//         // sys_tick_flag = 1; 
//          // Bật cờ cho Main loop chạy
//     }
// }

// int main(void)
// {
//     // 1. INIT
//      UART_Init();      // Init UART trước để debug nếu treo
//     //  I2C_Init();
//      MPU6050_Init();
    
//     // Config LED
//     DDRB |= 1 << LED;

//     // 3. START TIMER
//     Timer0_Init();
//      sei(); 

//     while (1) 
//     {
//         // // Chờ cờ ngắt (được bật mỗi ~10ms)
//         // if (sys_tick_flag == 1) 
//         // {
//         //     // sys_tick_flag = 0; // Xóa cờ

            
//         //     //   PORTB ^= (1 << PB5);
//         //       timer_counter=0; // Nháy đèn
//         //     // --- C. In ra Serial (Mỗi 100ms) ---
//         //     print_counter++;
//         //       sys_tick_flag = 0; // Xóa cờ
//         //     // if (print_counter >= 10) { 
//         //     //     print_counter = 0;
                
//         //     //     PORTB ^= (1 << PB5); // Nháy đèn
//         //     // }
//         // }
//     }
// }


/*
 * main_test_mpu.c
 * Kiểm tra đọc dữ liệu MPU6050 qua I2C và gửi về UART
 */


/*
 * Self-Balancing Robot Firmware
 * Method: Sampling Loop using Timer0 Overflow Interrupt
 * Timer0 Config: Normal Mode, Prescaler 64 -> Overflow every ~1ms
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "i2c.h"
#include "mpu6050.h"
#include "filter.h"
#include "pid.h"
#include "dc_motor.h"

// --- CONFIGURATION ---
// Không cần OCR vì dùng ngắt tràn (Overflow)
#define KP_VAL          15000L 
#define KI_VAL          80L    
#define KD_VAL          1200L  
#define TARGET_ANGLE    0      

// --- GLOBAL VARIABLES ---
volatile uint8_t sys_tick_flag = 0; // Cờ báo hiệu chạy PID
volatile uint8_t timer_counter = 0; // Biến đếm số lần tràn

char buffer[64];
MPU6050_t mpu_data;
CompFilter_t angle_filter;
PID_Config_t balance_pid;
int32_t gyro_offset_x = 0;
uint8_t print_counter = 0;

// --- UART FUNCTIONS ---
void UART_Init(void) {
    UBRR0H = 0; UBRR0L = 103; // 9600 baud
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void UART_Tx(char data) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = data; }
void UART_Print(char *str) { while (*str) UART_Tx(*str++); }

// --- TIMER0 INIT (THEO YÊU CẦU CỦA BẠN) ---
void Timer0_Init(void) {
    // 1. Normal Mode
    TCCR0A = 0x00; 
    
    // 2. Prescaler 64 (CS01=1, CS00=1)
    // T_overflow = (64 * 256) / 16MHz = 1.024 ms
    TCCR0B = (1 << CS01) | (1 << CS00);

    // 3. Reset Counter
    TCNT0 = 0;

    // 4. Cho phép ngắt tràn (Overflow Interrupt Enable)
    TIMSK0 |= (1 << TOIE0); 

    // LƯU Ý: Đã xóa while(1) và sei() ở đây để tránh treo và đúng chuẩn structure
}

// --- TIMER0 OVERFLOW ISR ---
// Hàm này chạy mỗi 1.024ms
ISR(TIMER0_OVF_vect) 
{
    timer_counter++;
    
    // Chúng ta cần chu kỳ 10ms.
    // 1.024ms * 10 = 10.24ms (Sai số chấp nhận được)
    if (timer_counter >= 10) {
        timer_counter = 0;  // Reset biến đếm
        sys_tick_flag = 1;  // Bật cờ cho Main loop chạy
    }
}

// --- CALIBRATION ---
void Calibrate_Gyro(void) {
    int32_t sum = 0;
    const int samples = 500;
    PORTB |= (1 << PB5); // LED ON
    for (int i = 0; i < samples; i++) {
        MPU6050_Read_All(&mpu_data);
        sum += mpu_data.Gyro_X_RAW;
        _delay_ms(2);
    }
    gyro_offset_x = sum / samples;
    PORTB &= ~(1 << PB5); // LED OFF
}

int main(void)
{
    // 1. Setup Drivers
    UART_Init();
    MPU6050_Init();
        
    DDRB |= (1 << PB5); // LED Pin
    // 2. Start Timer0
    Timer0_Init();
    
    // 3. Bật ngắt toàn cục (QUAN TRỌNG)
    sei(); 

    while (1) 
    {
        // Chờ cờ ngắt (được bật mỗi ~10ms)
        if (sys_tick_flag == 1) 
        {
            sys_tick_flag = 0; // Xóa cờ

            // --- A. Đọc MPU ---
            MPU6050_Read_All(&mpu_data);

            // --- B. Tính toán góc ---
            float angle_f = atan2(mpu_data.Ay, mpu_data.Az) * 57.296;
            
            // --- C. In ra Serial (Mỗi 100ms) ---
            print_counter++;
            if (print_counter >= 10) { 
                print_counter = 0;
                // In ra giá trị raw để kiểm tra
                sprintf(buffer, "Gx:%d | Ang:%d\r\n", (int)mpu_data.Gyro_X_RAW, (int)angle_f);
                UART_Print(buffer);
                PORTB ^= (1 << PB5); // Nháy đèn
            }
        }
    }
}
/*
 * PROJECT: Self Balancing Robot
 * HARDWARE: Arduino Uno (ATmega328P), MPU6050, L298N (12V), JGA25 Motors
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>      // Cho printf
#include <stdlib.h>     // Cho hàm abs()

// Include các driver (đảm bảo file header đúng tên)
#include "i2c.h"
#include "mpu6050.h"
#include "filter.h"
#include "pid.h"
#include "dc_motor.h"  

// --- CẤU HÌNH ---
#define SAMPLE_TIME_MS  5        // Chu kỳ lấy mẫu 5ms (200Hz)
#define BAUD            9600
#define MYUBRR          (F_CPU/16/BAUD-1)

// [QUAN TRỌNG] CẤU HÌNH ĐỘNG CƠ
// BÙ DEADZONE: Động cơ sẽ luôn chạy ít nhất là 600 (hoặc -600) khi có lệnh PID
// MAX LIMIT: Kẹp cứng ở 800 để bảo vệ Timer
#define PWM_DEADZONE    0      
#define PWM_MAX_LIMIT   800      

// --- BIẾN TOÀN CỤC ---
Kalman_t robotFilter;     
PID_Data_t balancePID;    

// Biến Debug (để in ra màn hình từ vòng lặp chính)
volatile int32_t debug_angle = 0;
volatile int32_t debug_pwm = 0;

// =============================================================
// 1. UART SETUP (Để dùng printf debug)
// =============================================================
void uart_init(void) {
    UBRR0H = (MYUBRR >> 8) & 0xFF;
    UBRR0L = MYUBRR & 0xFF;
    UCSR0B = (1 << TXEN0);  
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') uart_putchar('\r', stream);
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
    return 0;
}

static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// =============================================================
// 2. TIMER 2 SETUP (Ngắt 5ms cho hệ thống)
// =============================================================
void Timer2_Init_5ms(void) {
    TCCR2A = (1 << WGM21); // CTC Mode
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
    OCR2A = 77; // (16MHz / 1024 / 200Hz) - 1 = 77
    TIMSK2 = (1 << OCIE2A); // Enable Interrupt
}

// =============================================================
// 3. CHƯƠNG TRÌNH CHÍNH
// =============================================================
int main(void) {
    // A. Khởi tạo UART
    uart_init();
    stdout = &uart_output;
    printf("\n\n=== ROBOT STARTING ===\n");

    // B. Khởi tạo Driver
    i2c_init();
    printf("I2C... OK\n");

    MPU6050_Init();
    printf("MPU6050... OK\n");

    Motor_Init();
    printf("Motor Driver (12V L298N)... OK\n");

    // C. Khởi tạo Thuật toán
    Kalman_Init(&robotFilter);
    PID_init(&balancePID); 

    // --- PID TUNING (CÂN CHỈNH) ---
    // Vì đã cộng thêm 600 (Deadzone), Kp không cần quá lớn.
    // Nếu xe rung lắc mạnh: GIẢM Kp, TĂNG Kd.
    // Nếu xe đổ mà không dậy nổi: TĂNG Kp.
    balancePID.Kp = 2000;  // 10.0 (Thử giảm xuống một chút vì Deadzone đã mạnh)
    balancePID.Ki = 50;    // 0.5
    balancePID.Kd = 800;   // 8.0

    // =========================================================
    // D. CALIBRATION (Cân chỉnh điểm 0 trong 5 giây)
    // =========================================================
    printf(">> GIU XE THANG DUNG TRONG 5s...\n");
    _delay_ms(1000);

    long sum_angle = 0;
    int valid_samples = 0;

    for (int i = 0; i < 500; i++) { // Chạy tầm 2.5s thực tế
        MPU6050_ReadData(&mpu_data);
        
        // Chú ý: Đảm bảo filter.c của bạn đã dùng đúng biến Ax, Ay...
        int32_t ang = Kalman_Update(&robotFilter,&mpu_data, SAMPLE_TIME_MS);

        if (i > 100) { // Bỏ qua giai đoạn đầu
            sum_angle += ang;
            valid_samples++;
        }
        _delay_ms(SAMPLE_TIME_MS);
    }

    int32_t offset = sum_angle / valid_samples;
    balancePID.target_angle = offset; 

    printf(">> DONE! Offset: %ld mDeg\n", offset);
    printf(">> MOTOR ON IN 2s...\n");
    _delay_ms(2000);

    // =========================================================
    // E. RUN
    // =========================================================
    balancePID.error_sum = 0; 
    
    Timer2_Init_5ms(); // Bắt đầu đếm nhịp
    sei();             // Bật ngắt -> ISR chạy -> Xe bắt đầu cân bằng

    while (1) {
        // In thông số debug (PID output chưa cộng deadzone và final PWM)
        printf("Ang: %ld | PWM: %ld\n", debug_angle, debug_pwm);
        _delay_ms(100); 
    }
}

// =============================================================
// 4. ISR (VÒNG LẶP ĐIỀU KHIỂN 200Hz)
// =============================================================
ISR(TIMER2_COMPA_vect) {
    // 1. Đọc cảm biến
    MPU6050_ReadData(&mpu_data);

    // 2. Lọc Kalman
    int32_t current_angle = Kalman_Update(&robotFilter,&mpu_data, SAMPLE_TIME_MS);

    // 3. Tính PID
    // Kết quả pid_out chỉ là phần điều chỉnh (ví dụ: 10, 50, -100...)
    PID_calculator(&balancePID, current_angle, SAMPLE_TIME_MS);
    int32_t pid_out = balancePID.output;

    // 4. BÙ DEADZONE & TÍNH FINAL PWM
    int32_t final_pwm = 0;

    // Logic: Nếu PID muốn xe di chuyển, lập tức cộng thêm 600 để thắng ma sát
    if (pid_out > 0) {
        final_pwm = pid_out + PWM_DEADZONE; 
    } else if (pid_out < 0) {
        final_pwm = pid_out - PWM_DEADZONE;
    } else {
        final_pwm = 0;
    }

    // 5. Kẹp dòng (Saturation)
    // Đảm bảo không vượt quá 800 (giới hạn Timer1)
    if (final_pwm > PWM_MAX_LIMIT) final_pwm = PWM_MAX_LIMIT;
    else if (final_pwm < -PWM_MAX_LIMIT) final_pwm = -PWM_MAX_LIMIT;

    // 6. CHẾ ĐỘ BẢO VỆ (Safety Cutoff)
    // Nếu nghiêng quá 45 độ (45000 mDeg) -> Tắt máy ngay
    if (abs(current_angle) > 45000) {
        final_pwm = 0;
        balancePID.error_sum = 0; // Reset tích phân để khi dựng lên không bị giật
    }

    // 7. Điều khiển Motor
    Motor_Control(final_pwm, final_pwm);

    // 8. Lưu debug
    debug_angle = current_angle;
    debug_pwm = final_pwm;
}

/*
 * PROJECT: Self Balancing Robot - ATmega328P
 * FINAL INTEGRATION
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>      // Thư viện Input/Output chuẩn (cho printf)

// Include các driver tự viết
#include "i2c.h"
#include "mpu6050.h"
#include "filter.h"
#include "pid.h"
#include "dc_motor.h"   // Hoặc "motor.h" tùy tên file bạn lưu

// --- CẤU HÌNH ---
#define SAMPLE_TIME_MS 5        // Chu kỳ lấy mẫu 5ms (200Hz)
#define BAUD 9600               // Tốc độ Serial
#define MYUBRR (F_CPU/16/BAUD-1)// Tính toán thanh ghi Baudrate

// --- BIẾN TOÀN CỤC ---
Kalman_t robotFilter;     // Bộ lọc Kalman
PID_Data_t balancePID;    // Bộ điều khiển PID

// Biến chia sẻ để Main Loop đọc được dữ liệu từ ngắt ISR
volatile int32_t debug_angle = 0;
volatile int32_t debug_pwm = 0;

// =============================================================
// 1. CẤU HÌNH UART (Để dùng printf)
// =============================================================
void uart_init(void) {
    UBRR0H = (MYUBRR >> 8) & 0xFF;
    UBRR0L = MYUBRR & 0xFF;
    UCSR0B = (1 << TXEN0);  // Chỉ bật truyền (TX)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit
}

// Hàm gửi ký tự (dùng cho printf)
static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') uart_putchar('\r', stream);
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
    return 0;
}

// Khai báo stream cho printf
static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// =============================================================
// 2. CẤU HÌNH TIMER 2 (Nhịp tim 5ms)
// =============================================================
void Timer2_Init_5ms(void) {
    // Chế độ CTC (Clear Timer on Compare Match)
    TCCR2A = (1 << WGM21);
    // Prescaler = 1024
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
    // Giá trị so sánh: (16MHz / 1024 / 200Hz) - 1 = 77
    OCR2A = 77; 
    // Bật ngắt so sánh kênh A
    TIMSK2 = (1 << OCIE2A);
}

// =============================================================
// 3. CHƯƠNG TRÌNH CHÍNH
// =============================================================
int main(void) {
    // A. Khởi tạo UART & Printf đầu tiên để debug
    uart_init();
    stdout = &uart_output; // Trỏ printf về UART
    
    printf("\n\n=== ROBOT KHOI DONG ===\n");

    // B. Khởi tạo Phần cứng
    i2c_init();
    printf("I2C... OK\n");

    MPU6050_Init();
    printf("MPU6050... OK\n");

    Motor_Init(); // Timer1 cho PWM động cơ
    printf("Motor Driver... OK\n");

    // C. Khởi tạo Thuật toán
    Kalman_Init(&robotFilter);
    PID_init(&balancePID);

    // --- CÀI ĐẶT PID (TUNING) ---
    // Kp, Ki, Kd đã nhân với hệ số tỉ lệ (ví dụ 100)
    // Bạn sẽ thay đổi các số này khi chạy thử thực tế
    balancePID.Kp = 2000;  // Kp = 18.0
    balancePID.Ki = 80;    // Ki = 0.8
    balancePID.Kd = 1200;  // Kd = 12.0

    // =========================================================
    // D. GIAI ĐOẠN CALIBRATION (CÂN CHỈNH ĐIỂM 0)
    // =========================================================
    printf(">> GIU XE THANG DUNG! CAN CHINH TRONG 5 GIAY...\n");
    _delay_ms(1000); // Chờ 1s để tay bạn ổn định

    long sum_angle = 0;
    int valid_samples = 0;

    // Chạy vòng lặp giả lập 500 lần (khoảng 2.5 - 3 giây)
    for (int i = 0; i < 500; i++) {
        MPU6050_ReadData((MPU6050_Data*)&mpu_data);
        
        // Chạy Kalman để bộ lọc hội tụ
        int32_t ang = Kalman_Update(&robotFilter, (MPU6050_Data*)&mpu_data, SAMPLE_TIME_MS);

        // Bỏ qua 100 mẫu đầu tiên (khi bộ lọc chưa ổn định)
        if (i > 100) {
            sum_angle += ang;
            valid_samples++;
        }
        _delay_ms(SAMPLE_TIME_MS);
    }

    // Tính góc lệch trung bình
    int32_t offset = sum_angle / valid_samples;
    balancePID.target_angle = offset; // Đặt làm điểm cân bằng mới

    printf(">> XONG! Offset: %ld mDeg\n", offset);
    printf(">> MOTOR SE CHAY SAU 2 GIAY...\n");
    _delay_ms(2000);

    // =========================================================
    // E. BẮT ĐẦU VẬN HÀNH
    // =========================================================
    // Xóa tích phân PID cũ trước khi chạy
    balancePID.error_sum = 0;
    balancePID.last_error = 0;

    Timer2_Init_5ms(); // Bắt đầu đếm nhịp
    sei();             // Cho phép ngắt toàn cục -> ISR bắt đầu chạy

    // Vòng lặp chính chỉ dùng để in thông số (Debug)
    while (1) {
        printf("Ang: %ld | Tgt: %ld | PWM: %ld\n", 
               debug_angle, 
               balancePID.target_angle, 
               debug_pwm);
        
        // In chậm thôi để không ảnh hưởng hiệu năng vi điều khiển
        _delay_ms(100); 
    }
}

// =============================================================
// 4. NGẮT HỆ THỐNG (LOOP 200Hz)
// =============================================================
ISR(TIMER2_COMPA_vect) {
    // 1. Đọc cảm biến
    MPU6050_ReadData((MPU6050_Data*)&mpu_data);

    // 2. Tính góc nghiêng (Kalman)
    int32_t current_angle = Kalman_Update(&robotFilter, (MPU6050_Data*)&mpu_data, SAMPLE_TIME_MS);

    // 3. Tính toán PID
    PID_calculator(&balancePID, current_angle, SAMPLE_TIME_MS);
    int32_t pwm_output = balancePID.output;

    // 4. CHẾ ĐỘ BẢO VỆ (Safety Cut-off)
    // Nếu xe nghiêng quá 45 độ (45000 mDeg), ngắt động cơ ngay lập tức
    // Tránh việc xe ngã rồi mà bánh vẫn quay tít mù khói
    if (current_angle > 45000 || current_angle < -45000) {
        pwm_output = 0;
        // Reset luôn các thành phần PID để khi dựng xe lên không bị giật
        balancePID.error_sum = 0;
        balancePID.last_error = 0;
    }

    // 5. Xuất xung ra động cơ
    Motor_Control(pwm_output, pwm_output);

    // 6. Lưu biến để Main Loop in ra màn hình
    debug_angle = current_angle;
    debug_pwm = pwm_output;
}
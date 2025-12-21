#include "dc_motor.h"
#include <stdlib.h> // Dùng hàm abs()

// Định nghĩa các chân điều khiển chiều (Direction)
// PORTD: PD4, PD5, PD6, PD7
#define DIR_PORT PORTD
#define DIR_DDR  DDRD

// Mask bit để thao tác nhanh (tránh ảnh hưởng chân khác của PORTD)
#define L_IN1 (1 << 4)  // PD4
#define L_IN2 (1 << 5)  // PD5
#define R_IN3 (1 << 6)  // PD6
#define R_IN4 (1 << 7)  // PD7

// Mask cho tất cả các chân Dir
#define DIR_MASK (L_IN1 | L_IN2 | R_IN3 | R_IN4)

// Dead zone compensation cho L298N (motor không quay nếu PWM < 100)
#define MOTOR_DEADZONE 100

void Motor_Init(void) {
    // 1. Cấu hình chân PWM (PB1/OC1A và PB2/OC1B) là Output
    DDRB |= (1 << PB1) | (1 << PB2);

    // 2. Cấu hình chân Chiều (PD4, PD5, PD6, PD7) là Output
    DIR_DDR |= DIR_MASK;
    
    // Tắt động cơ ban đầu (Low hết các chân Dir)
    DIR_PORT &= ~DIR_MASK;

    // 3. Cấu hình Timer1 (16-bit) cho PWM 20kHz
    // Mode 14: Fast PWM, TOP = ICR1
    // Clear OC1A/OC1B on Compare Match (Non-Inverting mode)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    
    // Prescaler = 1 (Chạy trực tiếp 16MHz không chia)
    // WGM13 = 1, WGM12 = 1
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

    // Tính toán tần số: F_pwm = F_cpu / (Prescaler * (1 + TOP))
    // 20,000 = 16,000,000 / (1 * (1 + 799))
    ICR1 = MOTOR_MAX_PWM - 1; // 799

    // Khởi tạo duty cycle bằng 0
    OCR1A = 0;
    OCR1B = 0;
}

// [OPTIMIZATION]: static inline helper
// Hàm nội bộ để set chiều quay, giúp code chính gọn hơn
static inline void set_direction_left(int16_t speed) {
    if (speed > 0) {
        // Tiến: IN1 High, IN2 Low
        DIR_PORT = (DIR_PORT | L_IN1) & ~L_IN2;
    } else if (speed < 0) {
        // Lùi: IN1 Low, IN2 High
        DIR_PORT = (DIR_PORT & ~L_IN1) | L_IN2;
    } else {
        // Dừng: Phanh điện (High High) - Hiệu quả hơn cho balancing robot
        DIR_PORT |= (L_IN1 | L_IN2);
    }
}

static inline void set_direction_right(int16_t speed) {
    if (speed > 0) {
        // Tiến: IN3 High, IN4 Low
        DIR_PORT = (DIR_PORT | R_IN3) & ~R_IN4;
    } else if (speed < 0) {
        // Lùi: IN3 Low, IN4 High
        DIR_PORT = (DIR_PORT & ~R_IN3) | R_IN4;
    } else {
        // Dừng: Phanh điện (High High)
        DIR_PORT |= (R_IN3 | R_IN4);
    }
}

void Motor_Control(int16_t speed_L, int16_t speed_R) {
    // [OPTIMIZATION]: register
    // Đưa biến vào thanh ghi để xử lý nhanh nhất có thể
    register int16_t pwm_L = speed_L;
    register int16_t pwm_R = speed_R;

    // 1. Giới hạn tốc độ (Safety Clamp)
    if (pwm_L > MOTOR_MAX_PWM) pwm_L = MOTOR_MAX_PWM;
    else if (pwm_L < -MOTOR_MAX_PWM) pwm_L = -MOTOR_MAX_PWM;

    if (pwm_R > MOTOR_MAX_PWM) pwm_R = MOTOR_MAX_PWM;
    else if (pwm_R < -MOTOR_MAX_PWM) pwm_R = -MOTOR_MAX_PWM;

    // 2. Dead zone compensation
    // Nếu PWM < DEADZONE thì set = 0 (motor không đủ lực để quay)
    if (abs(pwm_L) < MOTOR_DEADZONE && pwm_L != 0) {
        pwm_L = (pwm_L > 0) ? MOTOR_DEADZONE : -MOTOR_DEADZONE;
    }
    if (abs(pwm_R) < MOTOR_DEADZONE && pwm_R != 0) {
        pwm_R = (pwm_R > 0) ? MOTOR_DEADZONE : -MOTOR_DEADZONE;
    }

    // 3. Cài đặt chiều quay (Direction)
    set_direction_left(pwm_L);
    set_direction_right(pwm_R);

    // 4. Cài đặt tốc độ (Magnitude)
    // Lấy trị tuyệt đối để nạp vào thanh ghi PWM (vì thanh ghi chỉ nhận số dương)
    OCR1A = (uint16_t)abs(pwm_L); // Motor Trái
    OCR1B = (uint16_t)abs(pwm_R); // Motor Phải
}
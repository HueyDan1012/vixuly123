#ifndef DC_MOTOR_H
#define DC_MOTOR_H
#include <stdint.h>
#include <avr/io.h>

// Cấu hình giới hạn PWM (Dựa trên Timer1 ICR1)
// Với F_CPU 16MHz và tần số 20kHz -> MAX_PWM = 800
#define MOTOR_MAX_PWM 800

// Hàm khởi tạo Timer1 và GPIO
void Motor_Init(void);

/**
 * @brief Điều khiển tốc độ 2 động cơ
 * @param speed_L Tốc độ trái (-800 đến 800)
 * @param speed_R Tốc độ phải (-800 đến 800)
 */
void Motor_Control(int16_t speed_L, int16_t speed_R);

#endif // DC_MOTOR_H
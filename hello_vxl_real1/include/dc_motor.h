/*
 * dc_motor.h
 * Driver for 2 DC Motors using Hardware PWM (Timer1)
 * Updated for PB1/PB2 pins
 */

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include <stdint.h>
#include <avr/io.h>

// --- CẤU HÌNH PHẦN CỨNG (HARDWARE CONFIG) ---

// Chân điều khiển chiều quay (Direction Pins)
#define MOTOR_DIR_PORT  PORTD
#define MOTOR_DIR_DDR   DDRD

// Motor Left (M1)
#define M1_IN1_PIN      PD4
#define M1_IN2_PIN      PD5
#define M1_PWM_PORT     PORTB
#define M1_PWM_DDR      DDRB
#define M1_PWM_PIN      PB1 // OC1A (Timer1 Channel A)

// Motor Right (M2)
#define M2_IN1_PIN      PD6
#define M2_IN2_PIN      PD7
#define M2_PWM_PORT     PORTB
#define M2_PWM_DDR      DDRB
#define M2_PWM_PIN      PB2 // OC1B (Timer1 Channel B)

// Giới hạn PWM
// Timer1 Mode 5 (8-bit Fast PWM) có đỉnh là 255 (0x00FF)
#define MAX_PWM         255
#define MIN_START_PWM  40   // Deadzone

// --- Function Prototypes ---
void Motor_Init(void);
void Motor_L_Control(int16_t speed);
void Motor_R_Control(int16_t speed);
void Motor_Stop(void);

#endif /* DC_MOTOR_H_ */
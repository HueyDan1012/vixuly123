#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;

    int32_t target_angle;
    int32_t error_sum;
    int32_t last_error;

    int32_t output; // PWM result 
} PID_Data_t;

extern void PID_calculator(PID_Data_t* pid, int32_t input_angle,uint16_t dt_ms);
extern void PID_init(PID_Data_t* pid);
#endif // PID_H
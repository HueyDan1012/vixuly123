#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// Hệ số nhân để giả lập số thực (Scaling Factor)
// Ví dụ: Góc 1.5 độ -> 1500 (SCALING_FACTOR = 1000)
#define PID_SCALING_FACTOR  1000L

typedef struct {
    // Các hệ số Kp, Ki, Kd cũng được nhân với SCALING_FACTOR
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;

    // Các biến lưu trữ trạng thái
    int32_t Setpoint;       // Điểm đặt (mong muốn)
    int32_t IntegratedError; // Tổng sai số (cho thành phần I)
    int32_t LastError;      // Sai số lần trước (cho thành phần D)
    
    // Giới hạn chống bão hòa (Anti-windup) cho thành phần I
    int32_t IntegLimit;     
    
    // Giới hạn đầu ra (ví dụ PWM max = 255)
    int32_t OutputLimit;

} PID_Config_t;

// Hàm khởi tạo
void PID_Init(PID_Config_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t outLimit);

// Hàm tính toán - Input và Output đều là số nguyên
int16_t PID_Compute(PID_Config_t *pid, int32_t input, int32_t setpoint);

#endif /* PID_H_ */
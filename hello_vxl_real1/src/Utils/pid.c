#include "pid.h"

#define MAX_PWM 800
#define MIN_PWM -800

#define MAX_I 400000

void PID_init(PID_Data_t* pid) {
    pid->Kp = 0;
    pid->Ki = 0;
    pid->Kd = 0;
    pid->error_sum=0;
    pid->last_error=0;
    pid->target_angle=0;

}

void PID_calculator(PID_Data_t* pid, int32_t input_angle,uint16_t dt_ms) {
    int32_t error = pid->target_angle - input_angle;


    int32_t p = pid->Kp * error;
    pid->error_sum += error * dt_ms;


    // Kẹp dòng tích phân (Anti-windup)
    if (pid->error_sum > MAX_I) pid->error_sum = MAX_I;
    else if (pid->error_sum < -MAX_I) pid->error_sum = -MAX_I;

    int32_t i = pid->Ki * pid->error_sum;
    int32_t d = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // 5. Tổng hợp Output
    // Vì Kp, Ki, Kd đã nhân 100, nên kết quả cần chia 100
    // Tuy nhiên, input_angle là milli-degree (lớn gấp 1000 lần độ).
    // Nên ta cần cân nhắc hệ số chia.
    
    // Công thức thực dụng cho xe cân bằng:
    // Output = (Kp*E + Ki*Sum + Kd*Delta) >> Scaling_Shift
    
    int32_t total = p + i + d;
    
    // Chia tỉ lệ để ra PWM hợp lý (Cần tune thực tế)
    // Giả sử chia 100 (tương ứng scale Kp)
    pid->output = total / 100; 

    // 6. Kẹp dòng Output (Saturation)
    if (pid->output > MAX_PWM) pid->output = MAX_PWM;
    else if (pid->output < MIN_PWM) pid->output = MIN_PWM;
}


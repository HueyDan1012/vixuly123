#include "pid.h"

void PID_Init(PID_Config_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t outLimit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->Setpoint = 0;
    pid->IntegratedError = 0;
    pid->LastError = 0;
    
    // Giới hạn output (ví dụ PWM 8-bit là 255)
    // Nhưng vì ta đang tính toán ở dạng Scaled, nên giới hạn cũng phải nhân lên
    pid->OutputLimit = outLimit * PID_SCALING_FACTOR;
    
    // Giới hạn tích phân thường lấy bằng Output Limit để tránh quá tải
    pid->IntegLimit = pid->OutputLimit; 
}

int16_t PID_Compute(PID_Config_t *pid, int32_t input, int32_t setpoint)
{
    // 1. Tính sai số (Error)
    // Input là góc thực tế (đã nhân 1000), Setpoint là góc mong muốn (0)
    int32_t error = setpoint - input;

    // 2. Thành phần P (Proportional)
    int32_t pTerm = pid->Kp * error; // Kết quả sẽ có hệ số Scaling^2

    // 3. Thành phần I (Integral)
    pid->IntegratedError += error;

    // Chống bão hòa (Anti-windup)
    if (pid->IntegratedError > pid->IntegLimit) pid->IntegratedError = pid->IntegLimit;
    else if (pid->IntegratedError < -pid->IntegLimit) pid->IntegratedError = -pid->IntegLimit;

    int32_t iTerm = pid->Ki * pid->IntegratedError; // Kết quả có Scaling^2

    // 4. Thành phần D (Derivative)
    // D = (Error hiện tại - Error quá khứ)
    int32_t dTerm = pid->Kd * (error - pid->LastError); // Kết quả có Scaling^2
    pid->LastError = error;

    // 5. Tổng hợp PID
    // Công thức: Output = P + I + D
    int32_t outputLong = pTerm + iTerm + dTerm;

    // 6. Khử hệ số Scaling (Down-scaling)
    // Vì Kp, Ki, Kd đã nhân 1000, và Input cũng nhân 1000
    // Nên outputLong đang gấp 1,000,000 lần thực tế (1000*1000).
    // Ta cần chia bớt để đưa về miền giá trị của PWM.
    
    // Tùy vào cách bạn định nghĩa K ban đầu. 
    // Nếu bạn coi Kp=1.5 là 1500, thì ta cần chia cho SCALING_FACTOR^2 nếu input cũng scale.
    // Tuy nhiên, để đơn giản và tránh tràn số 32-bit:
    // Ta thường chia cho SCALING_FACTOR ở đây để đưa về lại thang đo 1000x của PWM.
    
    outputLong = outputLong / PID_SCALING_FACTOR;

    // 7. Giới hạn đầu ra (Saturation)
    if (outputLong > pid->OutputLimit) outputLong = pid->OutputLimit;
    else if (outputLong < -pid->OutputLimit) outputLong = -pid->OutputLimit;

    // 8. Chuyển về giá trị thực cho PWM (chia tiếp cho Scaling Factor còn lại)
    // Ví dụ outputLong đang là 255000 -> chia 1000 -> 255
    return (int16_t)(outputLong / PID_SCALING_FACTOR);
}
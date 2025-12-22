/*
 * filter.c
 */

#include "filter.h"

// ================================================================
// 1. INTEGER COMPLEMENTARY FILTER
// ================================================================

void CompFilter_Init(CompFilter_t *filter, int32_t alpha_x1000) {
    filter->Angle = 0;
    filter->K_Gyro = alpha_x1000;           // Ví dụ: 980
    filter->K_Accel = 1000L - alpha_x1000;  // Ví dụ: 20
}

int32_t CompFilter_Update(CompFilter_t *filter, int32_t accel_angle, int32_t gyro_rate, int32_t dt_ms) {
    // Công thức gốc (Float): 
    // Angle = alpha * (Angle + Gyro * dt) + (1-alpha) * Accel
    
    // --- CHUYỂN ĐỔI SANG INTEGER ---
    
    // 1. Tính phần dự đoán từ Gyro (Integration)
    // gyro_rate: milli-độ/giây (ví dụ 50000 = 50 độ/s)
    // dt_ms: milli-giây (ví dụ 10ms)
    // Góc thay đổi = (rate * dt) / 1000
    int32_t gyro_delta = (gyro_rate * dt_ms) / 1000; 
    
    int32_t predicted_angle = filter->Angle + gyro_delta;

    // 2. Áp dụng trọng số (Mixing)
    // Chúng ta nhân tất cả lên để tránh chia số lẻ, sau đó chia tổng cho 1000
    // Công thức: (980 * Dự_đoán + 20 * Accel) / 1000
    
    int32_t term1 = filter->K_Gyro * predicted_angle;
    int32_t term2 = filter->K_Accel * accel_angle;
    
    filter->Angle = (term1 + term2) / 1000;

    return filter->Angle;
}

// ================================================================
// 2. KALMAN FILTER (HYBRID Wrapper)
// ================================================================
// Giữ nguyên thuật toán Float bên trong vì Kalman số nguyên rất khó ổn định
// nhưng giao tiếp bên ngoài giả lập Integer để đồng bộ code.

void Kalman_Init(Kalman_t *kalman) {
    kalman->Q_angle = 0.001f;
    kalman->Q_bias  = 0.003f;
    kalman->R_measure = 0.03f;
    kalman->Angle = 0.0f;
    kalman->Bias  = 0.0f;
    kalman->P[0][0] = 0.0f; kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f; kalman->P[1][1] = 0.0f;
}

int32_t Kalman_Update(Kalman_t *kalman, int32_t accel_angle, int32_t gyro_rate, int32_t dt_ms) {
    // 1. Chuyển đổi Input Integer -> Float nội bộ
    float dt = (float)dt_ms / 1000.0f;       // 10ms -> 0.01s
    float newAngle = (float)accel_angle / 1000.0f; // 2500 -> 2.5
    float newRate = (float)gyro_rate / 1000.0f;    // 50000 -> 50.0

    // --- THUẬT TOÁN KALMAN (GIỮ NGUYÊN) ---
    float rate = newRate - kalman->Bias;
    kalman->Angle += rate * dt;

    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    float y = newAngle - kalman->Angle;
    kalman->Angle += K[0] * y;
    kalman->Bias  += K[1] * y;

    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    // 2. Chuyển đổi Output Float -> Integer
    return (int32_t)(kalman->Angle * 1000.0f);
}
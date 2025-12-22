#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>

// Hệ số Scale: 1000
#define FILTER_SCALE 1000L

// --- COMPLEMENTARY FILTER (PURE INTEGER) ---
typedef struct {
    int32_t Angle;      // Đơn vị: milli-degree
    int32_t K_Gyro;     // Hệ số tin tưởng Gyro (Ví dụ: 980 cho 0.98)
    int32_t K_Accel;    // Hệ số tin tưởng Accel (Ví dụ: 20 cho 0.02)
                        // K_Gyro + K_Accel phải bằng 1000
} CompFilter_t;

// --- KALMAN FILTER (WRAPPER) ---
// Bên trong vẫn dùng float để giữ độ chính xác thuật toán
typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float Angle;
    float Bias;
    float P[2][2];
} Kalman_t;

// --- Function Prototypes ---

// 1. Integer Complementary Functions
// alpha: 0 đến 1000 (ví dụ 980 = 0.98)
void CompFilter_Init(CompFilter_t *filter, int32_t alpha_x1000);
int32_t CompFilter_Update(CompFilter_t *filter, int32_t accel_angle, int32_t gyro_rate, int32_t dt_ms);

// 2. Kalman Functions
void Kalman_Init(Kalman_t *kalman);
// Input/Output là int32_t, nhưng tính toán bên trong là float
int32_t Kalman_Update(Kalman_t *kalman, int32_t accel_angle, int32_t gyro_rate, int32_t dt_ms);

#endif /* FILTER_H_ */
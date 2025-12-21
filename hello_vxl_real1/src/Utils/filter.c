#include "filter.h"
#include "stdlib.h"

#define ALPHA_GYRO 98
#define ALPHA_ACCEL 2

#define RAD_TO_MDEG 57296L

static inline int32_t calculate_Accel_angle(volatile MPU6050_Data* data) {
    if (data->Az == 0)
    return 0;

    return (int32_t)(((int64_t)data->Ay * RAD_TO_MDEG) / data->Az);
}

void Complementary_Init(Complementary_t* filter ) {
    filter->current_deg = 0;
}

int32_t Complementary_Update(Complementary_t* filter, MPU6050_Data* data,uint16_t dt_ms){
    register int32_t accel_angle = calculate_Accel_angle(data);
    register int32_t gyro_rate = data->Gx;
    
// Tich phan Gyroscope
    int32_t gyro_term = filter->current_deg + (gyro_rate* dt_ms) / 1000;
// ket hop filter
filter->current_deg = (ALPHA_GYRO * gyro_term + ALPHA_ACCEL * accel_angle) / 100;

return filter->current_deg;

}

void Kalman_Init(Kalman_t* filter) {
    filter->angle = 0;
    filter->bias  = 0;
    filter->P00 = 0; filter->P01 = 0;
    filter->P10 = 0; filter->P11 = 0;

    // Tuning (Fixed-point representation) (Số học điểm tĩnh)
    // Q_angle = 0.001, Q_bias = 0.003, R = 0.03 / VD: 0.001*4096
    // Phuong sai
    filter->Q_angle = 4;   // "Nhiễu của quá trình".
    filter->Q_bias  = 12; // "Tốc độ trôi của Gyro".
    filter->R_measure = 123; //"Nhiễu của Gia tốc kế".
}

int32_t Kalman_Update(Kalman_t* filter, volatile MPU6050_Data* data,uint16_t dt_ms) {
    int32_t newAngle = calculate_Accel_angle(data); // Milli-degrees
    int32_t newRate  = data->Gx;          // Milli-degrees/s
    
    
    register int32_t dt_fp = (dt_ms * KALMAN_ONE) / 1000 ; // delta time – floating point
    register int32_t rate;
    register int32_t y,S;
    register int32_t K0,K1;


    // PREDICT 
    rate = (newRate << KALMAN_Q_SHIFT) - filter->bias ; // deg/s
    filter->angle += (rate * dt_fp) >> KALMAN_Q_SHIFT; // tích phân rate

    // Predict Covariance
    // P00 += dt * (dt*P11 - P01 - P10 + Q_angle)
    int32_t dt_P11 = (dt_fp * filter->P11) >> KALMAN_Q_SHIFT;
    
    filter->P00 += ((dt_fp * (dt_P11 - filter->P01 - filter->P10)) >> KALMAN_Q_SHIFT) + filter->Q_angle;
    filter->P01 -= dt_P11;
    filter->P10 -= dt_P11;
    filter->P11 += filter->Q_bias;


    // Update
    // Tính lệch
    y = ( newAngle << KALMAN_Q_SHIFT) - filter->angle;
    // S = P + R
    S = filter->P00 + filter->R_measure;
    if(S == 0) 
    {
        S = 1;
    }

    K0 = ((int64_t)filter->P00 << KALMAN_Q_SHIFT) / S; // Hệ số sửa góc
    K1 = ((int64_t)filter->P10 << KALMAN_Q_SHIFT) / S; // Hệ số sửa Bias

    filter->angle += (K0 * y) >> KALMAN_Q_SHIFT;
    filter->bias  += (K1 * y) >> KALMAN_Q_SHIFT;

    // Update Covariance   // giảm độ bất an
    int32_t P00_temp = filter->P00;
    int32_t P01_temp = filter->P01;

    
    filter->P00 -= (K0 * P00_temp) >> KALMAN_Q_SHIFT; // (Trừ đi lượng tin cậy vừa đạt được).
    filter->P01 -= (K0 * P01_temp) >> KALMAN_Q_SHIFT;
    filter->P10 -= (K1 * P00_temp) >> KALMAN_Q_SHIFT;
    filter->P11 -= (K1 * P01_temp) >> KALMAN_Q_SHIFT;

    // Return mill-degree
    return (filter->angle >> KALMAN_Q_SHIFT);
}   
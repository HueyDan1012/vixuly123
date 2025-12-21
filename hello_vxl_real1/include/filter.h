#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <mpu6050.h>

#define KALMAN_Q_SHIFT 12
#define KALMAN_ONE (1 << KALMAN_Q_SHIFT)

// Complementary Filter
typedef struct {
    int32_t current_deg;

} Complementary_t;

void Complementary_Init(Complementary_t* filter);
int32_t Complementary_Update(Complementary_t* filter, MPU6050_Data* data ,uint16_t dt_ms);

// Kalman Filter

typedef struct {
int32_t angle;
int32_t bias ;

int32_t P00,P01, P10,P11;

// Tuning
int32_t Q_angle;
int32_t Q_bias;
int32_t R_measure;
} Kalman_t;

int32_t Kalman_Update(Kalman_t* filter ,volatile MPU6050_Data* data,uint16_t dt_ms);
void Kalman_Init(Kalman_t* filter);
#endif // FILTER_H
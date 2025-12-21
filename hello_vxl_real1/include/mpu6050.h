#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>
#include "i2c.h"

#define MPU6050_ADDR         0x68

#define PWR_MGMT_1 0x6B
#define MPU_SMPLRT_DIV 0X19
#define MPU_CONFIG 0X1A
#define GYRO_CONFIG 0X1B
#define ACCEL_CONFIG 0X1C
#define WHO_AM_I 0x75

#define MPU_ACCEL_XOUT_H 0x3B
typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temperature_RAW;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;

    int32_t Ax,Ay,Az;
    int32_t Gx,Gy,Gz;

} MPU6050_Data;
extern volatile MPU6050_Data mpu_data;

void MPU6050_Init(void);
void MPU6050_ReadData(volatile MPU6050_Data* data);
#endif // MPU6050_H
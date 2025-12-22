#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include "i2c.h"

// --- MPU6050 Registers ---
#define MPU6050_ADDR         0x68 // Default I2C address (AD0 = GND)
#define SMPLRT_DIV_REG       0x19
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define ACCEL_XOUT_H_REG     0x3B // First data register
#define PWR_MGMT_1_REG       0x6B
#define WHO_AM_I_REG         0x75

// --- Configuration Constants ---
// Gyro Sensitivity (LSB per deg/s)
// FS_SEL=0: 131, FS_SEL=1: 65.5, FS_SEL=2: 32.8, FS_SEL=3: 16.4
#define GYRO_SENS_SCALE      65.5  // For +/- 500 deg/s range

// Accel Sensitivity (LSB per g)
// AFS_SEL=0: 16384, AFS_SEL=1: 8192, AFS_SEL=2: 4096, AFS_SEL=3: 2048
#define ACCEL_SENS_SCALE     8192.0 // For +/- 4g range

// --- Data Structure ---
typedef struct {
    // Raw 16-bit data from sensor
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Temperature_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // Converted Physical Float Data
    float Ax;       // units: g
    float Ay;       // units: g
    float Az;       // units: g
    float Gx;       // units: deg/s
    float Gy;       // units: deg/s
    float Gz;       // units: deg/s

} MPU6050_t;

// --- Function Prototypes ---
void MPU6050_Init(void);
void MPU6050_Read_All(MPU6050_t *DataStruct);

#endif /* MPU6050_H_ */
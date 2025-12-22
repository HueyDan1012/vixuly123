/*
 * mpu6050.c
 */

#include "mpu6050.h"
#include <math.h> // For any math if needed later, but mainly for float handling

void MPU6050_Init(void)
{
    // 1. Wake up the MPU6050 (It starts in sleep mode)
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0); // Address + Write bit
    I2C_Write(PWR_MGMT_1_REG);          // Register to write
    I2C_Write(0x00);                    // Write 0x00 to wake up
    I2C_Stop();

    // 2. Set Gyro Full Scale Range to +/- 500 deg/s
    // Register 0x1B (GYRO_CONFIG). Bit 3 and 4 control range.
    // 0x08 sets bits 0000 1000 => FS_SEL = 1
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(GYRO_CONFIG_REG);
    I2C_Write(0x08); 
    I2C_Stop();

    // 3. Set Accel Full Scale Range to +/- 4g
    // Register 0x1C (ACCEL_CONFIG). Bit 3 and 4 control range.
    // 0x08 sets bits 0000 1000 => AFS_SEL = 1
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(ACCEL_CONFIG_REG);
    I2C_Write(0x08); 
    I2C_Stop();
}

void MPU6050_Read_All(MPU6050_t *DataStruct)
{
    // Burst Read starting from ACCEL_XOUT_H (Register 0x3B)
    // We read 14 sequential bytes: 
    // Accel_X (H/L), Accel_Y (H/L), Accel_Z (H/L), Temp (H/L), Gyro_X (H/L)...

    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0); // Address + Write
    I2C_Write(ACCEL_XOUT_H_REG);        // Set pointer to first data register
    I2C_Start();                        // Repeated Start
    I2C_Write((MPU6050_ADDR << 1) | 1); // Address + Read

    // Read 14 bytes. Send ACK for the first 13, NACK for the last one.
    
    // --- ACCELEROMETER ---
    DataStruct->Accel_X_RAW = (int16_t)(I2C_Read_Ack() << 8 | I2C_Read_Ack());
    DataStruct->Accel_Y_RAW = (int16_t)(I2C_Read_Ack() << 8 | I2C_Read_Ack());
    DataStruct->Accel_Z_RAW = (int16_t)(I2C_Read_Ack() << 8 | I2C_Read_Ack());

    // --- TEMPERATURE (We usually ignore this for balancing, but must read it to advance pointer) ---
    DataStruct->Temperature_RAW = (int16_t)(I2C_Read_Ack() << 8 | I2C_Read_Ack());

    // --- GYROSCOPE ---
    DataStruct->Gyro_X_RAW  = (int16_t)(I2C_Read_Ack() << 8 | I2C_Read_Ack());
    DataStruct->Gyro_Y_RAW  = (int16_t)(I2C_Read_Ack() << 8 | I2C_Read_Ack());
    DataStruct->Gyro_Z_RAW  = (int16_t)(I2C_Read_Nack() << 8 | I2C_Read_Nack()); // NACK last byte

    I2C_Stop();

    // --- CONVERSION TO PHYSICAL QUANTITIES ---
    
    // 1. Convert Accel Raw to 'g' (gravity units)
    DataStruct->Ax = (float)DataStruct->Accel_X_RAW / ACCEL_SENS_SCALE;
    DataStruct->Ay = (float)DataStruct->Accel_Y_RAW / ACCEL_SENS_SCALE;
    DataStruct->Az = (float)DataStruct->Accel_Z_RAW / ACCEL_SENS_SCALE;

    // 2. Convert Gyro Raw to 'degrees per second'
    DataStruct->Gx = (float)DataStruct->Gyro_X_RAW / GYRO_SENS_SCALE;
    DataStruct->Gy = (float)DataStruct->Gyro_Y_RAW / GYRO_SENS_SCALE;
    DataStruct->Gz = (float)DataStruct->Gyro_Z_RAW / GYRO_SENS_SCALE;
}
#include "mpu6050.h"
// hệ số độ nhạy
static const int ACCEL_SCALE_FACTOR = 16384;
static const int GYRO_SCALE_FACTOR  = 131;
volatile MPU6050_Data mpu_data;
void MPU6050_Init(void) {
    i2c_init();

    // Wake up MPU6050
    i2c_start(MPU6050_ADDR << 1);  // Slave write address (0xD0)
    i2c_write(PWR_MGMT_1);
    i2c_write(0x00);
    i2c_stop();

    // Sample rate divider
    i2c_start(MPU6050_ADDR << 1);
    i2c_write(MPU_SMPLRT_DIV);
    i2c_write(0x07); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    i2c_stop();

    // Gyro config
    i2c_start(MPU6050_ADDR << 1);
    i2c_write(GYRO_CONFIG);
    i2c_write(0x00); // ±250 °/s
    i2c_stop();

    // Accel config
    i2c_start(MPU6050_ADDR << 1);
    i2c_write(ACCEL_CONFIG);
    i2c_write(0x00); // ±2 g
    i2c_stop();
}

void MPU6050_ReadData(volatile MPU6050_Data* data) {
    // Start write to set register address
    i2c_start(MPU6050_ADDR << 1);  // Slave write
    i2c_write(MPU_ACCEL_XOUT_H);

    // Repeated start for read
    i2c_start((MPU6050_ADDR << 1) | 0x01);  // Slave read

    // Read Accel_X
    uint8_t highByte = i2c_read_ack();
    uint8_t lowByte = i2c_read_ack();
    data->Accel_X = (int16_t)((highByte << 8) | lowByte);

    // Read Accel_Y
    highByte = i2c_read_ack();
    lowByte = i2c_read_ack();
    data->Accel_Y = (int16_t)((highByte << 8) | lowByte);

    // Read Accel_Z
    highByte = i2c_read_ack();
    lowByte = i2c_read_ack();
    data->Accel_Z = (int16_t)((highByte << 8) | lowByte);

    // Read Temperature
    highByte = i2c_read_ack();
    lowByte = i2c_read_ack();
    data->Temperature_RAW = (int16_t)((highByte << 8) | lowByte);

    // Read Gyro_X
    highByte = i2c_read_ack();
    lowByte = i2c_read_ack();
    data->Gyro_X = (int16_t)((highByte << 8) | lowByte);

    // Read Gyro_Y
    highByte = i2c_read_ack();
    lowByte = i2c_read_ack();
    data->Gyro_Y = (int16_t)((highByte << 8) | lowByte);

    // Read Gyro_Z
    highByte = i2c_read_ack();
    lowByte = i2c_read_nack();  // NACK for last byte
    data->Gyro_Z = (int16_t)((highByte << 8) | lowByte);

    i2c_stop();

    // Chuyển đổi giá trị thô sang giá trị vật lý
    data->Ax = ((int32_t)data->Accel_X * 1000) / ACCEL_SCALE_FACTOR;
    data->Ay = ((int32_t)data->Accel_Y * 1000) / ACCEL_SCALE_FACTOR;
    data->Az = ((int32_t)data->Accel_Z * 1000) / ACCEL_SCALE_FACTOR;

    data->Gx = ((int32_t)data->Gyro_X * 1000) / GYRO_SCALE_FACTOR;
    data->Gy = ((int32_t)data->Gyro_Y * 1000) / GYRO_SCALE_FACTOR;
    data->Gz = ((int32_t)data->Gyro_Z * 1000) / GYRO_SCALE_FACTOR;
}
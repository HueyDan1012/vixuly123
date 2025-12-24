#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz
#endif 

// I2C Clock Speed (Standard 100kHz or Fast 400kHz)
#define SCL_CLOCK 400000UL 
#define I2C_TIMEOUT 2000

// Function Prototypes
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Read_Ack(void);
uint8_t I2C_Read_Nack(void);

uint8_t I2C_IsError(void);
#endif /* I2C_H_ */
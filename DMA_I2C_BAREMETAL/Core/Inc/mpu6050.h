#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"


#define MPU6050_ADDR 0xD0


#define WHO_AM_I_REG  0x75
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43


void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel_Gyro_DMA(I2C_HandleTypeDef *hi2c);

extern uint8_t MPU6050_Data[14];

#endif

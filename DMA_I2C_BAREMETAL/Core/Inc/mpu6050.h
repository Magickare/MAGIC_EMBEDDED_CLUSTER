#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

/* MPU6050 I2C Address */
#define MPU6050_ADDR 0xD0

/* MPU6050 Register Addresses */
#define WHO_AM_I_REG  0x75
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43

/* Function Declarations */
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel_Gyro_DMA(I2C_HandleTypeDef *hi2c);

/* External Variables */
extern volatile uint8_t dma_complete; // Declare as extern
extern uint8_t mpu_data[14]; // Declare as extern

#endif

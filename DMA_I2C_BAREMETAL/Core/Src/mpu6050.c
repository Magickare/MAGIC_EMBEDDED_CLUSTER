#include "mpu6050.h"
#include <stdio.h>
#include "main.h"

/* Define global variables ONLY here */
uint8_t mpu_data[14]; // Global buffer for MPU6050 data
volatile uint8_t dma_complete = 0; // DMA completion flag

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;

    /* Read WHO_AM_I register */
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if (check == 0x68) {
        printf("MPU6050 detected successfully!\r\n");

        /* Wake up MPU6050 */
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    } else {
        printf("MPU6050 not found!\r\n");
    }
}

/* Start DMA-based Read */
void MPU6050_Read_Accel_Gyro_DMA(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, 1, mpu_data, 14);
}

/* DMA Callback */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        dma_complete = 1; // Indicate data received
    }
}

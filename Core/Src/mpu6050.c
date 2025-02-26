#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include <stdio.h>
#include "main.h"
#include <string.h>

#define MPU6050_H
#define MPU6050_ADDR 0xD0  // Change if using a different I2C address
#define ACCEL_XOUT_H 0x3B


extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
uint8_t mpu_data[14];
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);


void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;


    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if (check == 0x68) {
        printf("MPU6050 detected successfully!\r\n");


        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    } else {
        printf("MPU6050 not found!\r\n");
    }
}

void MPU6050_Read_Accel_Gyro(I2C_HandleTypeDef *hi2c) {
    uint8_t data[14];


    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, 1, data, 14, 100);


    int16_t accel_x = (int16_t)(data[0] << 8 | data[1]);
    int16_t accel_y = (int16_t)(data[2] << 8 | data[3]);
    int16_t accel_z = (int16_t)(data[4] << 8 | data[5]);

    int16_t gyro_x = (int16_t)(data[8] << 8 | data[9]);
    int16_t gyro_y = (int16_t)(data[10] << 8 | data[11]);
    int16_t gyro_z = (int16_t)(data[12] << 8 | data[13]);

    // Print values to UART
    printf("Accel X: %d, Y: %d, Z: %d | Gyro X: %d, Y: %d, Z: %d\r\n",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
}

void MPU6050_Read_Accel_Gyro_DMA(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, 1, mpu_data, 14);
}

volatile uint8_t dma_complete = 0;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        printf("DMA Callback Triggered!\r\n");

        int16_t accel_x = (int16_t)(mpu_data[0] << 8 | mpu_data[1]);
        int16_t accel_y = (int16_t)(mpu_data[2] << 8 | mpu_data[3]);
        int16_t accel_z = (int16_t)(mpu_data[4] << 8 | mpu_data[5]);

        int16_t gyro_x = (int16_t)(mpu_data[8] << 8 | mpu_data[9]);
        int16_t gyro_y = (int16_t)(mpu_data[10] << 8 | mpu_data[11]);
        int16_t gyro_z = (int16_t)(mpu_data[12] << 8 | mpu_data[13]);

        printf("DMA Accel X: %d, Y: %d, Z: %d | Gyro X: %d, Y: %d, Z: %d\r\n",
               accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

        dma_complete = 1;
    }
}




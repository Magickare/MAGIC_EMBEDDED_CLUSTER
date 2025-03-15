/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for MPU6050 with DMA callback function
  ******************************************************************************
  * @attention
  *
  * This code demonstrates interfacing with the MPU6050 sensor using I2C with DMA
  * and a callback function to handle the DMA transfer complete event. FreeRTOS is
  * used to periodically trigger sensor reads.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define MPU6050_ADDR         (0x68 << 1)  // MPU6050 I2C address (7-bit address shifted for HAL)
#define MPU6050_PWR_MGMT_1   0x6B         // Power management register address
#define MPU6050_ACCEL_XOUT_H 0x3B         // Starting register for accelerometer data

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId mpu6050TaskHandle;

/* Buffer for sensor data:
   6 bytes for accelerometer, 2 bytes for temperature, 6 bytes for gyroscope */
uint8_t mpu6050Data[14];

int16_t AccelX, AccelY, AccelZ;
int16_t Temperature;
int16_t GyroX, GyroY, GyroZ;

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartMPU6050Task(void const * argument);
void MPU6050_Init(void);
void MPU6050_Read(void);

/* USER CODE BEGIN 0 */
/* Initialize the MPU6050 sensor (wake it up) */
void MPU6050_Init(void)
{
  uint8_t data = 0;
  // Write 0 to the power management register to wake up MPU6050
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
                        &data, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Start a DMA-based read from the MPU6050 */
void MPU6050_Read(void)
{
  // Read 14 bytes starting from the ACCEL_XOUT_H register
  if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                           mpu6050Data, 14) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Callback function: Called when the DMA-based I2C read is complete */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    // Combine high and low bytes for each measurement
    AccelX = (int16_t)(mpu6050Data[0] << 8 | mpu6050Data[1]);
    AccelY = (int16_t)(mpu6050Data[2] << 8 | mpu6050Data[3]);
    AccelZ = (int16_t)(mpu6050Data[4] << 8 | mpu6050Data[5]);
    GyroX = (int16_t)(mpu6050Data[8] << 8 | mpu6050Data[9]);
    GyroY = (int16_t)(mpu6050Data[10] << 8 | mpu6050Data[11]);
    GyroZ = (int16_t)(mpu6050Data[12] << 8 | mpu6050Data[13]);

    // Format a message with the detection confirmation and sensor values
    char msg[150];
    int len = snprintf(msg, sizeof(msg),
                       "Detected successfully: AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d\r\n",
                       AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* Initialize the MPU6050 sensor */
  MPU6050_Init();

  /* Create the FreeRTOS tasks */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(mpu6050Task, StartMPU6050Task, osPriorityNormal, 0, 128);
  mpu6050TaskHandle = osThreadCreate(osThread(mpu6050Task), NULL);

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop (should never reach here) */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Enable DMA controller clock and configure DMA interrupts
  * @retval None
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA interrupts */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure LD2 Pin Output */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Configure B1 Pin as input with external interrupt */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure LD2 Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* FreeRTOS Default Task */
void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

/* FreeRTOS Task to read MPU6050 sensor data periodically */
void StartMPU6050Task(void const * argument)
{
  for(;;)
  {
    MPU6050_Read();    // Start DMA read from MPU6050
    osDelay(100);      // Delay between reads (adjust as needed)
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Optional: report the file name and line number
}
#endif /* USE_FULL_ASSERT */

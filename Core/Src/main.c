/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"
#include "frame.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;   /* defined in USB_DEVICE/App/usb_device.c */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static MPU6050_t mpu;

#define SAMPLE_PERIOD_MS  10U   /* 100 Hz, matches MPU6050_CONFIG_DEFAULT (1 kHz / (1+9)) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void stream_frame(uint8_t cfg, uint8_t seq, uint32_t t_us, const MPU6050_Raw_t *raw);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* Wait (max 3 s) for the PC to enumerate the CDC device, then give the
     terminal a moment to open the port so the first lines are not lost. */
  for (uint32_t t0 = HAL_GetTick();
       hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED && (HAL_GetTick() - t0) < 3000U; )
  {
    HAL_Delay(10);
  }
  HAL_Delay(1000);
  printf("\r\nMPU6050 demo\r\n");

  const MPU6050_Config_t cfg = MPU6050_CONFIG_DEFAULT;
  MPU6050_Status st = MPU6050_Init(&mpu, &hi2c1, 0U, &cfg);   /* AD0 = GND -> 0x68 */
  while (st != MPU6050_OK)
  {
    /* Keep reporting so the message is seen even if the terminal opens late */
    printf("MPU6050 init failed: %d (check wiring, 3.3V, AD0=GND)\r\n", (int)st);
    HAL_Delay(1000);
    st = MPU6050_Init(&mpu, &hi2c1, 0U, &cfg);
  }

  printf("Calibrating, keep the sensor still and flat...\r\n");
  if (MPU6050_Calibrate(&mpu, 500U) != MPU6050_OK)
  {
    printf("Calibration failed\r\n");
    Error_Handler();
  }
  printf("Streaming binary frames at 100 Hz\r\n");

  /* Free-running cycle counter -> microsecond timestamps */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  const uint8_t frame_cfg = (uint8_t)((unsigned)cfg.accel_range | ((unsigned)cfg.gyro_range << 2));
  uint8_t frame_seq = 0U;
  uint32_t next_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Fixed 100 Hz pacing, independent of how long each I2C read takes */
    next_tick += SAMPLE_PERIOD_MS;
    while ((int32_t)(HAL_GetTick() - next_tick) < 0)
    {
    }

    uint32_t t_us = DWT->CYCCNT / (SystemCoreClock / 1000000U);
    MPU6050_Raw_t raw;
    if (MPU6050_ReadRawCalibrated(&mpu, &raw) == MPU6050_OK)
    {
      stream_frame(frame_cfg, frame_seq, t_us, &raw);
    }
    else
    {
      MPU6050_RecoverBus(&mpu);   /* a gap in seq tells the PC a sample was lost */
    }
    frame_seq++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USB CDC TX helpers.
   CDC_Transmit_FS() does not copy the data, so every buffer handed to it must
   stay valid until the transfer completes (TxState back to 0). */
static int cdc_busy(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  return hcdc == NULL || hcdc->TxState != 0U;
}

/* Binary frame: never blocks; if the previous transfer is still running the
   frame is dropped (the PC sees the gap in seq). */
static void stream_frame(uint8_t cfg, uint8_t seq, uint32_t t_us, const MPU6050_Raw_t *raw)
{
  static uint8_t frame_buf[FRAME_LEN];

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || cdc_busy())
  {
    return;
  }
  Frame_Pack(frame_buf, cfg, seq, t_us, raw);
  (void)CDC_Transmit_FS(frame_buf, FRAME_LEN);
}

/* printf -> USB CDC (overrides the weak _write in syscalls.c). Text can be
   mixed with the binary stream: the PC parser resyncs on the 0xAA 0x55 header
   and ASCII text never contains it. */
int _write(int file, char *ptr, int len)
{
  static uint8_t tx_buf[256];
  (void)file;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return len;   /* no host connected: drop the data instead of blocking */
  }

  int sent = 0;
  while (sent < len)
  {
    uint16_t chunk = (uint16_t)((len - sent) > (int)sizeof(tx_buf) ? sizeof(tx_buf) : (len - sent));
    uint32_t t0 = HAL_GetTick();

    while (cdc_busy())
    {
      if ((HAL_GetTick() - t0) > 20U)
      {
        return sent;   /* USB stalled: give up on the rest */
      }
    }
    for (uint16_t i = 0; i < chunk; i++)
    {
      tx_buf[i] = (uint8_t)ptr[sent + i];
    }
    if (CDC_Transmit_FS(tx_buf, chunk) != USBD_OK)
    {
      return sent;
    }
    sent += chunk;
  }
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

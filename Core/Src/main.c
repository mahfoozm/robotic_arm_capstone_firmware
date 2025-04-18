/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Encoder Angle Reader
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <stdio.h>
#include <string.h>

#define CMD_QUEUE_SIZE 24
#define CMD_MAX_LENGTH 64
#define ENCODER_CPR 4000.0f  // Encoder: 1000 PPR * 4 (quadrature)
#define GEAR_RATIO 10.0f
#define COUNTS_PER_JOINT_REV (ENCODER_CPR * GEAR_RATIO)  // 4000 × 10 = 40000
#define DEGREES_PER_TURN 360.0f

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

uint8_t rxChar;
static char commandBuffer[CMD_MAX_LENGTH];
static uint16_t commandIndex = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

int32_t getEncoderPos(uint8_t motorNum)
{
  if (motorNum == 1)
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  else if (motorNum == 2)
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
  return 0;
}

float calculateAngle(int32_t counts)
{
  float angle = ((float)counts / COUNTS_PER_JOINT_REV) * DEGREES_PER_TURN;
  if (angle > 180.0f)
      angle -= 360.0f;
  else if (angle < -180.0f)
      angle += 360.0f;
  return angle;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if ((rxChar == '\n') || (rxChar == '\r'))
    {
      commandBuffer[commandIndex] = '\0';
      if (commandIndex > 0)
      {
        uint8_t motorNum = 0;
        if (sscanf(commandBuffer, "GET:E:%hhu", &motorNum) == 1 && (motorNum == 1 || motorNum == 2)) {
          int32_t pos = getEncoderPos(motorNum);
          float angle = calculateAngle(pos);
          char resp[80];
          snprintf(resp, sizeof(resp), "Encoder %u: %ld counts -> %.2f deg\r\n", motorNum, (long)pos, angle);
          HAL_UART_Transmit(&huart2, (uint8_t*)resp, strlen(resp), 100);
        }
      }
      commandIndex = 0;
    }
    else
    {
      if (commandIndex < CMD_MAX_LENGTH - 1)
      {
        commandBuffer[commandIndex++] = rxChar;
      }
    }
    HAL_UART_Receive_IT(&huart2, &rxChar, 1);
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart2, &rxChar, 1);

  __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset encoder 1
  __HAL_TIM_SET_COUNTER(&htim1, 0); // Reset encoder 2

  const char *halo = "************ Hello Joaquin, Welcome Back ************\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)halo, strlen(halo), 100);

  const char *initMsg = "[Encoder Angle Reader Initialized]\r\nSend: GET:E:<1 or 2>\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)initMsg, strlen(initMsg), 100);

  while (1) {}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

static void MX_TIM1_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim1, &sConfig);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef   sConfig = {0};
  TIM_MasterConfigTypeDef   sMasterConfig = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = UINT32_MAX;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  /* noise filter: ignore pulses < 3 timer‑ticks */
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 3;

  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 3;

  HAL_TIM_Encoder_Init(&htim2, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_10) {
    // Index pulse: re‑zero encoder #1
    __HAL_TIM_SET_COUNTER(&htim2, 0);
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Custom assert handling
}
#endif

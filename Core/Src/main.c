/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This firmware drives the stepper motor and reads the encoder via TIM2.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CMD_QUEUE_SIZE 24
#define CMD_MAX_LENGTH 64

typedef struct {
    char commands[CMD_QUEUE_SIZE][CMD_MAX_LENGTH];
    volatile int head;
    volatile int tail;
} CommandQueue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// set to 400 steps currently on motor driver, gear ratio is 20
#define STEPS_PER_REV 400 * 20
#define STEPS_PER_DEGREE (STEPS_PER_REV / 360.0f)
#define MIN_STEP_INTERVAL_US 10
#define MAX_STEP_INTERVAL_US 100000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// None
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static int32_t motor1_stepsRemaining = 0;
static int32_t motor2_stepsRemaining = 0;

static float motor1_speedFactor = 1.0f;
static float motor2_speedFactor = 1.0f;

static uint32_t motor1_nextStepTime_us = 0;
static uint32_t motor2_nextStepTime_us = 0;

static GPIO_PinState motor1_direction = GPIO_PIN_SET;
static GPIO_PinState motor2_direction = GPIO_PIN_SET;

static bool motor1_resetting = false;
static bool motor2_resetting = false;

uint8_t rxChar;
static char commandBuffer[CMD_MAX_LENGTH];
static uint16_t commandIndex = 0;
CommandQueue cmdQueue = { .head = 0, .tail = 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
static inline void delay_us(uint32_t us);
void readLine(char *line, size_t maxLen);
void parseCommand(const char *line);
int32_t getEncoderPos(uint8_t motorNum);
void doMotorStepping(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Initialize the DWT cycle counter for microsecond delays.
  */
void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// BROKEN USING HAL_DELAY() INSTEAD FIX THIS IF NECESSARY
///**
//  * @brief  Provides a delay (in microseconds) using the DWT cycle counter.
//  * @param  us: Delay in microseconds.
//  */
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief Get the current time in microseconds using DWT.
 * @return Current time in microseconds.
 */
uint32_t get_micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

int isQueueEmpty(CommandQueue *q) {
    return (q->head == q->tail);
}

int isQueueFull(CommandQueue *q) {
    return (((q->tail + 1) % CMD_QUEUE_SIZE) == q->head);
}

int flushQueue(CommandQueue *q)
{
    q->head = 0;
    q->tail = 0;
    return 0;
}

int enqueueCommand(CommandQueue *q, const char *cmd) {
    if (isQueueFull(q)) {
        return -1;
    }
    strncpy(q->commands[q->tail], cmd, CMD_MAX_LENGTH - 1);
    q->commands[q->tail][CMD_MAX_LENGTH - 1] = '\0';
    q->tail = (q->tail + 1) % CMD_QUEUE_SIZE;
    return 0;
}

int dequeueCommand(CommandQueue *q, char *cmd, size_t maxLen) {
    if (isQueueEmpty(q)) {
        return -1;
    }
    strncpy(cmd, q->commands[q->head], maxLen - 1);
    cmd[maxLen - 1] = '\0';
    q->head = (q->head + 1) % CMD_QUEUE_SIZE;
    return 0;
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
                if (enqueueCommand(&cmdQueue, commandBuffer) != 0)
                {
                    // TODO: handle full queue
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

/* USER CODE END 0 */
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
  DWT_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart2, &rxChar, 1);

  const char *initMsg = "(+) initialized, waiting for commands...\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)initMsg, strlen(initMsg), 100);

/**
* @brief Get the encoder count for the specified motor.
*/
int32_t getEncoderPos(uint8_t motorNum)
{
  if (motorNum == 1)
    return __HAL_TIM_GET_COUNTER(&htim2);
  else if (motorNum == 2)
    return __HAL_TIM_GET_COUNTER(&htim1);
  return 0;
}


/**
 * @brief Parse a command line and act on it.
 * Commands:
 *   M<motor_number>:<value>
 *   GET:E:<motor_number>
 *   RESET:<motor_number>  // Added RESET command
 */
void parseCommand(const char *line)
{
    char resp[80];
    if (line[0] == 'M')
    {
        uint8_t motorNum = line[1] - '0'; // single-digit motor ID

        char *colon1 = strchr(line, ':');
        if (!colon1)
        {
            const char *err = "ERR: no colon found\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
            return;
        }
        char *colon2 = strchr(colon1 + 1, ':');
        if (!colon2)
        {
            const char *err = "ERR: second colon not found\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
            return;
        }

        float movement = strtof(colon1 + 1, NULL);
        float speedVal = strtof(colon2 + 1, NULL);

        if (movement > 1.0f) movement = 1.0f;
        if (movement < -1.0f) movement = -1.0f;
        if (speedVal > 1.0f) speedVal = 1.0f;
        if (speedVal < 0.0f) speedVal = 0.0f;

        int32_t steps = (int32_t)llroundf(fabsf(movement) * STEPS_PER_REV);
        GPIO_PinState dir = (movement >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        float speedFactor = fabsf(speedVal);

        if (motorNum == 1)
        {
            motor1_stepsRemaining = steps;
            motor1_direction = dir;
            motor1_speedFactor = speedFactor;
            sprintf(resp, "Command: M1 => %ld steps, dir=%s, speed=%.2f\n",
                    (long)steps, (dir==GPIO_PIN_SET)?"fwd":"rev", speedFactor);
            HAL_UART_Transmit(&huart2, (uint8_t*)resp, strlen(resp), 100);
        }
        else if (motorNum == 2)
        {
            motor2_stepsRemaining = steps;
            motor2_direction = dir;
            motor2_speedFactor = speedFactor;
            sprintf(resp, "Command: M2 => %ld steps, dir=%s, speed=%.2f\n",
                    (long)steps, (dir==GPIO_PIN_SET)?"fwd":"rev", speedFactor);
            HAL_UART_Transmit(&huart2, (uint8_t*)resp, strlen(resp), 100);
        }
        else
        {
            const char *err2 = "ERR: motorNum invalid\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)err2, strlen(err2), 100);
        }
    }
    else if (line[0] == 'H')
    {
        flushQueue(&cmdQueue);
        motor1_stepsRemaining = 0;
        motor2_stepsRemaining = 0;
        motor1_resetting = false;
        motor2_resetting = false;
        const char *msg = "Command queue cleared and motors stopped\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }
    else if (strncmp(line, "GET:E:", 6) == 0)
    {
        uint8_t motorNum = 0;
        sscanf(line, "GET:E:%hhu", &motorNum);
        int32_t pos = getEncoderPos(motorNum);
        sprintf(resp, "E:%u:POS=%ld\n", (unsigned)motorNum, (long)pos);
        HAL_UART_Transmit(&huart2, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strncmp(line, "RESET:", 6) == 0)
    {
        uint8_t motorNum = line[6] - '0';
        if (motorNum == 1)
        {
            motor1_resetting = true;
            motor1_speedFactor = 1.0f;
            sprintf(resp, "Resetting motor 1 to zero position\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)resp, strlen(resp), 100);
        }
        else if (motorNum == 2)
        {
            motor2_resetting = true;
            motor2_speedFactor = 1.0f;
            sprintf(resp, "Resetting motor 2 to zero position\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)resp, strlen(resp), 100);
        }
        else
        {
            const char *err = "ERR: Invalid motor number for RESET\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
        }
    }
    else
    {
        char dbgMsg[CMD_MAX_LENGTH + 30];
        sprintf(dbgMsg, "ERR: Unknown command: '%s'\n", line);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbgMsg, strlen(dbgMsg), 100);
    }
}

/**
 * @brief Perform stepping for each motor based on motorX_speed.
 * Called repeatedly in the main loop.
 *
 * This is a simplistic approach that sets a step frequency
 * proportional to |speed|, toggles direction, etc.
 */
/**
 * @brief Perform stepping for each motor based on motorX_speed or reset mode.
 * Called repeatedly in the main loop.
 */
void doMotorStepping(void)
{
    uint32_t now_us = get_micros();

    if (motor1_resetting)
    {
        int32_t pos = getEncoderPos(1);
        if (pos == 0)
        {
            motor1_resetting = false;
        }
        else if (now_us >= motor1_nextStepTime_us)
        {
            GPIO_PinState dir = (pos > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            HAL_GPIO_WritePin(Motor1_DIR_GPIO_Port, Motor1_DIR_Pin, dir);
            HAL_GPIO_WritePin(Motor1_PUL_GPIO_Port, Motor1_PUL_Pin, GPIO_PIN_SET);
            delay_us(10);  // 10 us pulse width
            HAL_GPIO_WritePin(Motor1_PUL_GPIO_Port, Motor1_PUL_Pin, GPIO_PIN_RESET);

            uint32_t interval_us = MAX_STEP_INTERVAL_US - (uint32_t)(motor1_speedFactor * (MAX_STEP_INTERVAL_US - MIN_STEP_INTERVAL_US));
            if (interval_us < MIN_STEP_INTERVAL_US) interval_us = MIN_STEP_INTERVAL_US;
            motor1_nextStepTime_us = now_us + interval_us;
        }
    }
    else if (motor1_stepsRemaining > 0 && now_us >= motor1_nextStepTime_us)
    {
        HAL_GPIO_WritePin(Motor1_DIR_GPIO_Port, Motor1_DIR_Pin, motor1_direction);
        HAL_GPIO_WritePin(Motor1_PUL_GPIO_Port, Motor1_PUL_Pin, GPIO_PIN_SET);
        delay_us(10);
        HAL_GPIO_WritePin(Motor1_PUL_GPIO_Port, Motor1_PUL_Pin, GPIO_PIN_RESET);

        motor1_stepsRemaining--;
        uint32_t interval_us = MAX_STEP_INTERVAL_US - (uint32_t)(motor1_speedFactor * (MAX_STEP_INTERVAL_US - MIN_STEP_INTERVAL_US));
        if (interval_us < MIN_STEP_INTERVAL_US) interval_us = MIN_STEP_INTERVAL_US;
        motor1_nextStepTime_us = now_us + interval_us;
    }

    if (motor2_resetting)
    {
        int32_t pos = getEncoderPos(2);
        if (pos == 0)
        {
            motor2_resetting = false;
        }
        else if (now_us >= motor2_nextStepTime_us)
        {
            GPIO_PinState dir = (pos > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            HAL_GPIO_WritePin(Motor2_DIR_GPIO_Port, Motor2_DIR_Pin, dir);
            HAL_GPIO_WritePin(Motor2_PUL_GPIO_Port, Motor2_PUL_Pin, GPIO_PIN_SET);
            delay_us(10);
            HAL_GPIO_WritePin(Motor2_PUL_GPIO_Port, Motor2_PUL_Pin, GPIO_PIN_RESET);

            uint32_t interval_us = MAX_STEP_INTERVAL_US - (uint32_t)(motor2_speedFactor * (MAX_STEP_INTERVAL_US - MIN_STEP_INTERVAL_US));
            if (interval_us < MIN_STEP_INTERVAL_US) interval_us = MIN_STEP_INTERVAL_US;
            motor2_nextStepTime_us = now_us + interval_us;
        }
    }
    else if (motor2_stepsRemaining > 0 && now_us >= motor2_nextStepTime_us)
    {
        HAL_GPIO_WritePin(Motor2_DIR_GPIO_Port, Motor2_DIR_Pin, motor2_direction);
        HAL_GPIO_WritePin(Motor2_PUL_GPIO_Port, Motor2_PUL_Pin, GPIO_PIN_SET);
        delay_us(10);
        HAL_GPIO_WritePin(Motor2_PUL_GPIO_Port, Motor2_PUL_Pin, GPIO_PIN_RESET);

        motor2_stepsRemaining--;
        uint32_t interval_us = MAX_STEP_INTERVAL_US - (uint32_t)(motor2_speedFactor * (MAX_STEP_INTERVAL_US - MIN_STEP_INTERVAL_US));
        if (interval_us < MIN_STEP_INTERVAL_US) interval_us = MIN_STEP_INTERVAL_US;
        motor2_nextStepTime_us = now_us + interval_us;
    }
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	char cmd[CMD_MAX_LENGTH];
	if (dequeueCommand(&cmdQueue, cmd, sizeof(cmd)) == 0) {
		char echoMsg[80];
		sprintf(echoMsg, "Received command: %s\n", cmd);
		HAL_UART_Transmit(&huart2, (uint8_t*)echoMsg, strlen(echoMsg), 100);
		parseCommand(cmd);
	}

	doMotorStepping();
    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|Motor2_PUL_Pin|Motor2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor1_PUL_Pin|Motor1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Motor2_PUL_Pin Motor2_DIR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Motor2_PUL_Pin|Motor2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_PUL_Pin Motor1_DIR_Pin */
  GPIO_InitStruct.Pin = Motor1_PUL_Pin|Motor1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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

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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define DEAD_BAND_LOW	1948
#define DEAD_BAND_HIGH	2148

//Global joint speed
float global_angular_speed = 25.0f;

const uint32_t LOOP_INTERVAL_MS = 20;
uint32_t last_loop_tick = 0;

//Current joint position
float angle_base = 90.0f; //0 to 180
float angle_shoulder = 0.0f; //0 to 90
float angle_elbow = 0.0f;//0 to 90
float angle_wrist = 45.0f; //0 to 90
float angle_grab = 30.0f; //0 to 60

// Mechanical joint boundary limits
#define BASE_MIN        0.0f
#define BASE_MAX      180.0f
#define SHOULDER_MIN    0.0f
#define SHOULDER_MAX   90.0f
#define ELBOW_MIN       0.0f
#define ELBOW_MAX      90.0f
#define WRIST_MIN       0.0f
#define WRIST_MAX      90.0f
#define GRAB_MIN        0.0f
#define GRAB_MAX       60.0f

uint8_t rx_byte = 0;
char rx_buffer[32];
uint8_t rx_index = 0;
volatile uint8_t frame_ready = 0;

//Helper: Dynamically sample any ADC channel with 55.5 cycles sampling time
uint16_t Read_ADC_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 5);
  uint16_t val = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return val;
}

// Helper: Convert angle (0-180 deg) to pulse width (500-2500 us)
uint16_t angle_to_pulse(float angle)
{
  if (angle < 0.0f) angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  return (uint16_t)(500.0f + (angle / 180.0f) * 2000.0f);
}

// Helper: Update all 5 hardware timer PWM channels
void Update_Servo_PWM(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle_to_pulse(angle_base));     // PA0
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle_to_pulse(angle_shoulder)); // PA1
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle_to_pulse(angle_elbow));    // PA2
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, angle_to_pulse(angle_wrist));    // PA3
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, angle_to_pulse(angle_grab));     // PB6
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 1. Calibrate ADC
    HAL_ADCEx_Calibration_Start(&hadc1);

    // 2. Start PWM generation on all 5 servo channels
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    // 3. Move servos to default initial angles
    Update_Servo_PWM();

    // 4. Ensure LEDs start OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);

    // 5. Start background non-blocking UART reception
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Mode1
	  if (frame_ready){
		  int b, s, e, w, g;
		  if (sscanf(rx_buffer, "<%d,%d,%d,%d,%d>", &b, &s, &e, &w, &g) == 5){
			  // Enforce boundary safety limits on incoming BLE commands
			  angle_base     = (b < BASE_MIN)     ? BASE_MIN     : (b > BASE_MAX)     ? BASE_MAX     : (float)b;
			  angle_shoulder = (s < SHOULDER_MIN) ? SHOULDER_MIN : (s > SHOULDER_MAX) ? SHOULDER_MAX : (float)s;
			  angle_elbow    = (e < ELBOW_MIN)    ? ELBOW_MIN    : (e > ELBOW_MAX)    ? ELBOW_MAX    : (float)e;
			  angle_wrist    = (w < WRIST_MIN)    ? WRIST_MIN    : (w > WRIST_MAX)    ? WRIST_MAX    : (float)w;
			  angle_grab     = (g < GRAB_MIN)     ? GRAB_MIN     : (g > GRAB_MAX)     ? GRAB_MAX     : (float)g;

			  // Apply updated angles to hardware immediately
			  Update_Servo_PWM();

			  // Toggle PB12 LED to confirm packet receipt
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

			  // Send full acknowledgment frame back to PC
			  char ack_msg[48];
			  int len = snprintf(ack_msg, sizeof(ack_msg), "ACK:%d,%d,%d,%d,%d\r\n",
			                     (int)angle_base, (int)angle_shoulder, (int)angle_elbow, (int)angle_wrist, (int)angle_grab);
			  HAL_UART_Transmit(&huart1, (uint8_t*)ack_msg, len, 50);
		  }
		  frame_ready = 0;
	  }
	  //Mode2
	  if ((HAL_GetTick() - last_loop_tick) >= LOOP_INTERVAL_MS)
	      {
	        last_loop_tick = HAL_GetTick();
	        float step = global_angular_speed * (LOOP_INTERVAL_MS / 1000.0f);
	        uint8_t any_input_active = 0;

	        // Sample 4 Joystick Analog Channels
	        uint16_t adc_base     = Read_ADC_Channel(ADC_CHANNEL_7); // PA7
	        uint16_t adc_shoulder = Read_ADC_Channel(ADC_CHANNEL_6); // PA6
	        uint16_t adc_elbow    = Read_ADC_Channel(ADC_CHANNEL_8); // PB0
	        uint16_t adc_wrist    = Read_ADC_Channel(ADC_CHANNEL_9); // PB1

	        // --- Base Joint (PA7) ---
	        if (adc_base > DEAD_BAND_HIGH) {
	          angle_base -= step;
	          if (angle_base < BASE_MIN) angle_base = BASE_MIN;
	          any_input_active = 1;
	        } else if (adc_base < DEAD_BAND_LOW) {
	          angle_base += step;
	          if (angle_base > BASE_MAX) angle_base = BASE_MAX;
	          any_input_active = 1;
	        }

	        // --- Shoulder Joint (PA6) ---
	        if (adc_shoulder > DEAD_BAND_HIGH) {
	          angle_shoulder -= step;
	          if (angle_shoulder < SHOULDER_MIN) angle_shoulder = SHOULDER_MIN;
	          any_input_active = 1;
	        } else if (adc_shoulder < DEAD_BAND_LOW) {
	          angle_shoulder += step;
	          if (angle_shoulder > SHOULDER_MAX) angle_shoulder = SHOULDER_MAX;
	          any_input_active = 1;
	        }

	        // --- Elbow Joint (PB0) ---
	        if (adc_elbow > DEAD_BAND_HIGH) {
	          angle_elbow += step;
	          if (angle_elbow > ELBOW_MAX) angle_elbow = ELBOW_MAX;
	          any_input_active = 1;
	        } else if (adc_elbow < DEAD_BAND_LOW) {
	          angle_elbow -= step;
	          if (angle_elbow < ELBOW_MIN) angle_elbow = ELBOW_MIN;
	          any_input_active = 1;
	        }

	        // --- Wrist Joint (PB1) ---
	        if (adc_wrist > DEAD_BAND_HIGH) {
	          angle_wrist += step;
	          if (angle_wrist > WRIST_MAX) angle_wrist = WRIST_MAX;
	          any_input_active = 1;
	        } else if (adc_wrist < DEAD_BAND_LOW) {
	          angle_wrist -= step;
	          if (angle_wrist < WRIST_MIN) angle_wrist = WRIST_MIN;
	          any_input_active = 1;
	        }

	        // --- Active-Low Gripper Buttons (PB3 & PB4) ---
	        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
	          angle_grab += step;
	          if (angle_grab > GRAB_MAX) angle_grab = GRAB_MAX;
	          any_input_active = 1;
	        }
	        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) {
	          angle_grab -= step;
	          if (angle_grab < GRAB_MIN) angle_grab = GRAB_MIN;
	          any_input_active = 1;
	        }

	        // Update PWM outputs if manual controls modified the positions
	        if (any_input_active) {
	          Update_Servo_PWM();
	          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	        } else {
	          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	        }
	      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Non-blocking UART frame assembler (<...>)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rx_byte == '<')
    {
      rx_index = 0;
      rx_buffer[rx_index++] = rx_byte;
    }
    else if (rx_byte == '>')
    {
      if (rx_index < sizeof(rx_buffer) - 1)
      {
        rx_buffer[rx_index++] = rx_byte;
        rx_buffer[rx_index] = '\0';
        frame_ready = 1;
      }
    }
    else if (rx_index > 0 && rx_index < sizeof(rx_buffer) - 2)
    {
      rx_buffer[rx_index++] = rx_byte;
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
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

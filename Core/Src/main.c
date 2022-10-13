/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1_Port GPIOC
#define LED1_Pin  GPIO_PIN_0
#define LED2_Port GPIOC
#define LED2_Pin  GPIO_PIN_1
#define LED3_Port GPIOA
#define LED3_Pin  GPIO_PIN_7
#define LED4_Port GPIOB
#define LED4_Pin  GPIO_PIN_0
#define LED5_Port GPIOA
#define LED5_Pin  GPIO_PIN_4
#define LED6_Port GPIOA
#define LED6_Pin  GPIO_PIN_6
#define LED7_Port GPIOA
#define LED7_Pin  GPIO_PIN_0
#define LED8_Port GPIOA
#define LED8_Pin  GPIO_PIN_5

#define CLK_SPD HAL_RCC_GetSysClockFreq()

#define C0 16.35
#define CS0 17.32
#define Db0 17.32
#define D0 18.35
#define DS0 19.45
#define Eb0 19.45
#define E0 20.60
#define F0 21.83
#define FS0 23.12
#define Gb0 23.12
#define G0 24.50
#define GS0 25.96
#define Ab0 25.96
#define A0 27.50
#define AS0 29.14
#define B0 30.87
#define C1 32.70
#define CS1 34.65
#define D1 36.71
#define DS1 38.89
#define E1 41.20
#define F1 43.65
#define FS1 46.25
#define G1 49.00
#define GS1 51.91
#define A1 55.00
#define AS1 58.27
#define B1 61.74
#define C2 65.41
#define CS2 69.30
#define D2 73.42
#define DS2 77.78
#define E2 82.41
#define F2 87.31
#define FS2 92.50
#define G2 98.00
#define GS2 103.83
#define A2 110.00
#define AS2 116.54
#define B2 123.47
#define C3 130.81
#define CS3 138.59
#define D3 146.83
#define DS3 155.56
#define E3 164.81
#define F3 174.61
#define FS3 185.00
#define G3 196.00
#define GS3 207.65
#define A3 220.00
#define AS3 233.08
#define B3 246.94
#define C4 261.63
#define CS4 277.18
#define D4 293.66
#define DS4 311.13
#define E4 329.63
#define F4 349.23
#define FS4 369.99
#define G4 392.00
#define GS4 415.30
#define A4 440.00
#define AS4 466.16
#define B4 493.88
#define C5 523.25
#define CS5 554.37
#define D5 587.33
#define DS5 622.25
#define E5 659.25
#define F5 698.46
#define FS5 739.99
#define G5 783.99
#define GS5 830.61
#define A5 880.00
#define AS5 932.33
#define B5 987.77
#define C6 1046.50
#define CS6 1108.73
#define D6 1174.66
#define DS6 1244.51
#define E6 1318.51
#define F6 1396.91
#define FS6 1479.98
#define G6 1567.98
#define GS6 1661.22
#define A6 1760.00
#define AS6 1864.66
#define B6 1975.53
#define C7 2093.00
#define CS7 2217.46
#define D7 2349.32
#define DS7 2489.02
#define E7 2637.02
#define F7 2793.83
#define FS7 2959.96
#define G7 3135.96
#define GS7 3322.44
#define A7 3520.00
#define AS7 3729.31
#define B7 3951.07
#define C8 4186.01
#define CS8 4434.92
#define D8 4698.63
#define DS8 4978.03
#define E8 5274.04
#define F8 5587.65
#define FS8 5919.91
#define G8 6271.93
#define GS8 6644.88
#define A8 7040.00
#define AS8 7458.62
#define B8 7902.13

#define REST_FREQ 25000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON(port, pin) HAL_GPIO_WritePin(port, pin, 1)
#define LED_OFF(port, pin) HAL_GPIO_WritePin(port, pin, 0)

#define SET_FREQ(f) __HAL_TIM_SET_AUTORELOAD(&htim2, (CLK_SPD / f) - 1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t BPM = 120;
const uint16_t WN = 1000 * 60 / BPM * 4;
const uint16_t HN = WN / 2;
const uint16_t QN = HN / 2;
const uint16_t EN = QN / 2;
const uint16_t SN = EN / 2;
const uint16_t TN = SN / 2;

// Current LED turned on (1-8)
uint8_t LED = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SET_LED(double f)
{
  // Turn off old LED if it's on
  if      (LED == 0) ;
  else if (LED == 1) LED_OFF(LED1_Port, LED1_Pin);
  else if (LED == 2) LED_OFF(LED2_Port, LED2_Pin);
  else if (LED == 3) LED_OFF(LED3_Port, LED3_Pin);
  else if (LED == 4) LED_OFF(LED4_Port, LED4_Pin);
  else if (LED == 5) LED_OFF(LED5_Port, LED5_Pin);
  else if (LED == 6) LED_OFF(LED6_Port, LED6_Pin);
  else if (LED == 7) LED_OFF(LED7_Port, LED7_Pin);

  LED_OFF(LED8_Port, LED8_Pin);

  if (f == REST_FREQ) return;

  // Turn on new LED depending on note
  if (f == C0 || f == C1 || f == C2 || f == C3 || f == C4 || f == C5 || f == C6 || f == C7 || f == C8){
    LED_ON(LED1_Port, LED1_Pin);
    LED = 1;
  }
  else if (f == CS0 || f == CS1 || f == CS2 || f == CS3 || f == CS4 || f == CS5 || f == CS6 || f == CS7 || f == CS8){
    LED_ON(LED1_Port, LED1_Pin);
    LED_ON(LED8_Port, LED8_Pin);
    LED = 1;
  }

  else if (f == D0 || f == D1 || f == D2 || f == D3 || f == D4 || f == D5 || f == D6 || f == D7 || f == D8){
    LED_ON(LED2_Port, LED2_Pin);
    LED = 2;
  }
  else if(f == DS0 || f == DS1 || f == DS2 || f == DS3 || f == DS4 || f == DS5 || f == DS6 || f == DS7 || f == DS8){
    LED_ON(LED2_Port, LED2_Pin);
    LED_ON(LED8_Port, LED8_Pin);
    LED = 2;
  }

  else if (f == E0 || f == E1 || f == E2 || f == E3 || f == E4 || f == E5 || f == E6 || f == E7 || f == E8){
    LED_ON(LED3_Port, LED3_Pin);
    LED = 3;
  }

  else if (f == F0 || f == F1 || f == F2 || f == F3 || f == F4 || f == F5 || f == F6 || f == F7 || f == F8){
    LED_ON(LED4_Port, LED4_Pin);
    LED = 4;
  }
  else if(f == FS0 || f == FS1 || f == FS2 || f == FS3 || f == FS4 || f == FS5 || f == FS6 || f == FS7 || f == FS8){
    LED_ON(LED4_Port, LED4_Pin);
    LED_ON(LED8_Port, LED8_Pin);
    LED = 4;
  }

  else if (f == G0 || f == G1 || f == G2 || f == G3 || f == G4 || f == G5 || f == G6 || f == G7 || f == G8){
    LED_ON(LED5_Port, LED5_Pin);
    LED = 5;
  }
  else if(f == GS0 || f == GS1 || f == GS2 || f == GS3 || f == GS4 || f == GS5 || f == GS6 || f == GS7 || f == GS8){
    LED_ON(LED5_Port, LED5_Pin);
    LED_ON(LED8_Port, LED8_Pin);
    LED = 5;
  }

  else if (f == A0 || f == A1 || f == A2 || f == A3 || f == A4 || f == A5 || f == A6 || f == A7 || f == A8){
    LED_ON(LED6_Port, LED6_Pin);
    LED = 6;
  }
  else if (f == AS0 || f == AS1 || f == AS2 || f == AS3 || f == AS4 || f == AS5 || f == AS6 || f == AS7 || f == AS8){
    LED_ON(LED6_Port, LED6_Pin);
    LED_ON(LED8_Port, LED8_Pin);
    LED = 6;
  }

  else if (f == B0 || f == B1 || f == B2 || f == B3 || f == B4 || f == B5 || f == B6 || f == B7 || f == B8){
    LED_ON(LED7_Port, LED7_Pin);
    LED = 7;
  }

}

void REST(double d)
{
  SET_FREQ(REST_FREQ);
  SET_LED(REST_FREQ);
  HAL_Delay(d);
}

void NOTE(double f, double d){
  SET_FREQ(f);
  SET_LED(f);
  HAL_Delay(d);
}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  while (1)
  {
	REST(EN);
	NOTE(G4, EN);
	NOTE(A4, EN);
	NOTE(B4, EN);
	NOTE(D5, EN);
	NOTE(C5, EN);
	NOTE(C5, EN);
	NOTE(E5, EN);
	NOTE(D5, EN);

	NOTE(D5, EN);
	NOTE(G5, EN);
	NOTE(FS5, EN);
	NOTE(G5, EN);
	NOTE(D5, EN);
	NOTE(B4, EN);
	NOTE(G4, EN);
	NOTE(A4, EN);
	NOTE(B4, EN);

	NOTE(C5, EN);
	NOTE(D5, EN);
	NOTE(E5, EN);
	NOTE(D5, EN);
	NOTE(C5, EN);
	NOTE(B4, EN);
	NOTE(A4, EN);
	NOTE(B4, EN);
	NOTE(G4, EN);

	NOTE(FS4, EN);
	NOTE(G4, EN);
	NOTE(A4, EN);
	NOTE(D4, EN);
	NOTE(FS4, EN);
	NOTE(A4, EN);
	NOTE(C5, EN);
	NOTE(B4, EN);
	NOTE(A4, EN);

	NOTE(B4, EN);
	NOTE(G4, EN);
	NOTE(A4, EN);
	NOTE(B4, EN);
	NOTE(D5, EN);
	NOTE(C5, EN);
	NOTE(C5, EN);
	NOTE(E5, EN);
	NOTE(D5, EN);

	NOTE(D5, EN);
	NOTE(G5, EN);
	NOTE(FS5, EN);
	NOTE(G5, EN);
	NOTE(D5, EN);
	NOTE(B4, EN);
	NOTE(G4, EN);
	NOTE(A4, EN);
	NOTE(B4, EN);

	NOTE(E4, EN);
	NOTE(D5, EN);
	NOTE(C5, EN);
	NOTE(B4, EN);
	NOTE(A4, EN);
	NOTE(G4, EN);
	NOTE(D4, EN);
	NOTE(G4, EN);
	NOTE(FS4, EN);

	NOTE(G4, EN);
	NOTE(B4, EN);
	NOTE(D5, EN);
	NOTE(G5, EN);
	NOTE(D5, EN);
	NOTE(B4, EN);
	NOTE(G4, EN*3);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = CLK_SPD / 300;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = htim2.Init.Period * 0.01;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  htim2.Init.Prescaler = 0;
  htim2.Init.Period = CLK_SPD / 300;
  sConfigOC.Pulse = htim2.Init.Period * 0.01;
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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

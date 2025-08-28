/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PWM Generator and Measurement with UART Control
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include "string.h"
#include "math.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TIM1 için clock ayarları (PWM üretimi)
#define TIM1_CLK_HZ         (170000000UL)     // APB2 timer clock (170 MHz)
#define TIM1_PSC_VALUE      (170UL)           // Prescaler değeri: 170MHz / 170 = 1MHz

// TIM2 için clock ayarları (Input Capture) - YÜKSEK FREKANS İÇİN
#define TIM2_CLK_HZ         (170000000UL)     // APB1 timer clock (170 MHz)
#define TIM2_PSC_VALUE      (17UL)            // Prescaler değeri: 170MHz / 17 = 10MHz
#define TICK_HZ             (TIM2_CLK_HZ / TIM2_PSC_VALUE)   // 10 MHz
#define TICK_TO_US(x)       ((float)(x) * (1000000.0f / (float)TICK_HZ))
#define DIFF_TICKS(a,b)     (((b)>=(a)) ? ((b)-(a)) : (0xFFFFFFFFu - (a) + (b) + 1u))

// UART buffer size
#define UART_RX_BUFFER_SIZE 100
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t changed = 0;

typedef enum {  // sabit değişkenleri numaralandırarak kullandım
    IC_WAIT_RISE = 0,
    IC_WAIT_FALL,
    IC_WAIT_NEXT_RISE
} ic_state_t;

static volatile ic_state_t ic_state = IC_WAIT_RISE;
static volatile uint32_t t_rise1 = 0;
static volatile uint32_t t_fall  = 0;
static volatile uint32_t t_rise2 = 0;
static volatile uint32_t high_ticks   = 0;
static volatile uint32_t period_ticks = 0;

/* Ölçüm sonuçları */
volatile float frequency_hz = 0.0f;
volatile float duty_percent = 0.0f;
volatile uint32_t pulse_us  = 0;

/* PWM parametreleri */
static uint32_t current_frequency = 15000;  // Başlangıç frekansı (Hz)
static uint32_t current_duty = 25;          // Başlangıç duty cycle (%)

/* UART buffers */
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_char;
static volatile uint8_t uart_rx_index = 0;
static volatile uint8_t uart_command_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static inline void IC_SetPolarity_Rising(void) {
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
}

static inline void IC_SetPolarity_Falling(void) {
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
}

void IC_Start(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    IC_SetPolarity_Rising();
    ic_state = IC_WAIT_RISE;

    // Input capture için timer'ı yüksek hızda başlattım
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    // Debug mesajı
    char msg[] = "Input Capture started at 10MHz resolution\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void PWM_UpdateParameters(uint32_t frequency, uint32_t duty_percent_val)
{
    // Frekans sınırlaması kontrolü
    if (frequency > 75000) frequency = 75000;  // Maksimum 75kHz
    if (frequency < 1) frequency = 1;            // Minimum 1Hz
    if (duty_percent_val > 99) duty_percent_val = 99;  // Maksimum %99
    if (duty_percent_val < 1) duty_percent_val = 1;    // Minimum %1

    // HESAPLAMA SATIRLARI
    // Timer clock = 170MHz / prescaler(170) = 1MHz
    // Period = (Timer_Clock / Desired_Frequency) - 1
    uint32_t period = (1000000 / frequency) - 1; // ARR değeri (10MHz bazında)
    uint32_t pulse = ((period + 1) * duty_percent_val) / 100;  // CCR değeri

    // PWM parametrelerini güncelledim
    __HAL_TIM_SET_AUTORELOAD(&htim1, period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);

    // Global değişkenleri güncelledim
    current_frequency = frequency;
    current_duty = duty_percent_val;

    // Debug için UART'a gerçek hesaplanan değerleri gönderdim
    char debug[150];
    sprintf(debug, "DEBUG: period=%lu, pulse=%lu, actual_freq=%.2f Hz\r\n",
            period, pulse, 1000000.0f/(period+1));
    HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 100);
}

void UART_SendMeasurements(void)
{
    char buffer[200];
    sprintf(buffer, "\r\n--- PWM Measurements ---\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Set Frequency: %lu Hz, Set Duty: %lu%%\r\n", current_frequency, current_duty);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Measured Frequency: %.2f Hz\r\n", frequency_hz);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Measured Duty Cycle: %.2f%%\r\n", duty_percent);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Pulse Width: %lu us\r\n", pulse_us);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "------------------------\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}

void UART_SendHelp(void)
{
    char help_text[] =
        "\r\n=== PWM Generator Control ===\r\n"
        "Commands:\r\n"
        "  F <value>  - Set frequency (1-500000 Hz)\r\n"
        "  D <value>  - Set duty cycle (1-99%)\r\n"
        "  M          - Show measurements\r\n"
        "  H          - Show this help\r\n"
        "\r\n Examples: \r\n"
        "  F45000     - Set frequency to 45000 Hz\r\n"
        "  D75        - Set duty cycle to 75%\r\n";

    HAL_UART_Transmit(&huart2, (uint8_t*)help_text, strlen(help_text), 1000);
}

void Process_UART_Command(void)
{
    if (!uart_command_ready) return;

    uart_command_ready = 0;
    uart_rx_buffer[uart_rx_index] = '\0'; // Null

    char command = uart_rx_buffer[0];
    uint32_t value = 0;

    if (uart_rx_index > 1) {
        value = atoi((char*)&uart_rx_buffer[1]);  // atoi stringi int çevirir
    }

    switch (command) {
        case 'F':
        case 'f':
            if (value >= 1 && value <= 500000) {
                PWM_UpdateParameters(value, current_duty);
                char response[100];
                sprintf(response, "Frequency set to: %lu Hz\r\n", value);
                HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
                changed = 1;
            } else {
                char error[] = "Error: Frequency must be between 1-500000 Hz\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)error, strlen(error), 100);
            }
            break;

        case 'D':
        case 'd':
            if (value >= 1 && value <= 99) {
                PWM_UpdateParameters(current_frequency, value);
                char response[100];
                sprintf(response, "Duty cycle set to: %lu%%\r\n", value);
                HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
                changed = 1;
            } else {
                char error[] = "Error: Duty cycle must be between 1-99%\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)error, strlen(error), 100);
            }
            break;

        case 'M':
        case 'm':
            UART_SendMeasurements();
            break;

        case 'H':
        case 'h':
            UART_SendHelp();
            break;

        default:
            {
                char error[] = "Unknown command. Send 'H' for help.\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)error, strlen(error), 100);
            }
            break;
    }

    uart_rx_index = 0; // Reset buffer
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* TIM2 CH1 ISR callback */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return; //pointer TIM_HandleTypeDef TypeDeften instance konumunu al
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //cap değişkenını Hal kütüphane fonksşyonuna ata

    switch (ic_state)
    {
    case IC_WAIT_RISE:
        t_rise1 = cap;
        IC_SetPolarity_Falling();
        ic_state = IC_WAIT_FALL;
        break;

    case IC_WAIT_FALL:
        t_fall = cap;
        high_ticks = DIFF_TICKS(t_rise1, t_fall);
        IC_SetPolarity_Rising();
        ic_state = IC_WAIT_NEXT_RISE;
        break;

    case IC_WAIT_NEXT_RISE:
        t_rise2 = cap;
        period_ticks = DIFF_TICKS(t_rise1, t_rise2);

        if (period_ticks != 0) {
            frequency_hz = ((float)TICK_HZ / (float)period_ticks);
            duty_percent = 100.0f * ((float)high_ticks / (float)period_ticks);
        } else {
            frequency_hz = 0.0f;
            duty_percent = 0.0f;
        }

        pulse_us = (uint32_t)(TICK_TO_US(high_ticks) + 0.5f);

        /* Devamlı ölçüm için */
        t_rise1 = t_rise2;
        IC_SetPolarity_Falling();
        ic_state = IC_WAIT_FALL;
        break;

    default:
        IC_SetPolarity_Rising();
        ic_state = IC_WAIT_RISE;
        break;
    }
}

/* UART RX Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (uart_rx_char == '\r' || uart_rx_char == '\n') {
            if (uart_rx_index > 0) {
                uart_command_ready = 1;
            }
        } else if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1)) {
            uart_rx_buffer[uart_rx_index++] = uart_rx_char;
        }

        // Geri gönder terminale yazdır
        HAL_UART_Transmit(&huart2, &uart_rx_char, 1, 10);

        // Continue receiving
        HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);
    }
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Başlangıç PWM parametrelerini ayarla
  PWM_UpdateParameters(current_frequency, current_duty);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Input capture başlat
  IC_Start();

  // UART karşılama mesajı
  char welcome[] = "\r\n=== STM32G474 PWM Generator (FIXED) ===\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)welcome, strlen(welcome), 100);
  UART_SendHelp();

  // UART interrupt başlat
  HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Process_UART_Command();

    // Eğer parametreler değiştiyse ölçüm sonuçlarını gönder
    if (changed) {
        HAL_Delay(100);  // PWM'nin stabilize olması için bekle
        UART_SendMeasurements();
        changed = 0;
    }

    /* USER CODE BEGIN 3 */
    HAL_Delay(50);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief TIM1 Initialization Function - DÜZELTİLDİ!
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;  // 170-1 olucak yoksa nyquist örneklemesine giriyor
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function - DÜZELTİLDİ!
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 17-1;  // DÜZELTİLDİ: 10MHz timer clock için
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  // GPIOA0 AF1 for TIM2_CH1 - Input Capture için
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

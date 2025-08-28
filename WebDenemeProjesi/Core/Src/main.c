/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_ASCII    1   // 1: ASCII debug çıktısı; 0: binary DMA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static volatile uint8_t dbg_gate = 0;  // çok fazla yazmayı engellemek için
volatile uint8_t uart_ready = 1;       // UART DMA hazır bayrağı

#define SINE_LEN       256
#define ADC_BUF_LEN    1024

volatile uint16_t adc_buf[ADC_BUF_LEN];  // ADC DMA buffer
uint16_t dac_wave[SINE_LEN];             // DAC sinüs dalga tablosu
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ADC'yi güvenli şekilde yeniden başlat
static void Restart_ADC(void)
{
    HAL_ADC_Stop_DMA(&hadc3);
    (void)HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    memset((void*)adc_buf, 0, sizeof(adc_buf));
    (void)HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buf, ADC_BUF_LEN);
}

// ADC hata callback'i
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3) {
        Restart_ADC();
    }
}

// Sinüs dalga tablosunu doldur (DAC için 12-bit, 0-4095 aralığında)
void Fill_SineWave(void)
{
    const int mid = 2048;   // ~1.65V merkez (3.3V/2)
    const int amp = 1500;   // güvenli genlik (maksimum çıkış ~2.9V)

    for (int i = 0; i < SINE_LEN; i++)
    {
        float s = sinf(2.0f * M_PI * (float)i / (float)SINE_LEN);
        int v   = mid + (int)(amp * s);
        if (v < 0)    v = 0;
        if (v > 4095) v = 4095;
        dac_wave[i] = (uint16_t)v;
    }
}

// Debug fonksiyonu - ADC verilerini analiz ederek TeraTerm'e yazdır
static void UART_Print_ADC_Analysis(const uint16_t *src, size_t count)
{
    char line[80];

    // İstatistik analizi
    uint16_t local_min = 4095, local_max = 0;
    uint32_t local_sum = 0;

    for (size_t i = 0; i < count; i++)
    {
        if (src[i] < local_min) local_min = src[i];
        if (src[i] > local_max) local_max = src[i];
        local_sum += src[i];

        // İlk 8 değeri detaylı yazdır (voltaj = adc_val * 3300 / 4095 mV)
        if (i < 8) {
            uint32_t voltage_mv = (src[i] * 3300) / 4095;
            int n = snprintf(line, sizeof(line), "ADC[%u]: %u (%u.%03uV)\r\n",
                           (unsigned)i, (unsigned)src[i],
                           (unsigned)(voltage_mv/1000), (unsigned)(voltage_mv%1000));
            if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
        }
    }

    uint16_t local_avg = (uint16_t)(local_sum / count);

    // İstatistikleri voltaj cinsinden yazdır
    uint32_t min_mv = (local_min * 3300) / 4095;
    uint32_t max_mv = (local_max * 3300) / 4095;
    uint32_t avg_mv = (local_avg * 3300) / 4095;

    int n = snprintf(line, sizeof(line),
                    "MIN:%u(%u.%03uV) MAX:%u(%u.%03uV) AVG:%u(%u.%03uV)\r\n",
                    (unsigned)local_min, (unsigned)(min_mv/1000), (unsigned)(min_mv%1000),
                    (unsigned)local_max, (unsigned)(max_mv/1000), (unsigned)(max_mv%1000),
                    (unsigned)local_avg, (unsigned)(avg_mv/1000), (unsigned)(avg_mv%1000));
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 150);

    // Ayıraç
    HAL_UART_Transmit(&huart2, (uint8_t*)"================================\r\n", 34, 50);
}
/*
// Binary mod için ham veri yazdırma (şu anda kullanılmıyor)
static void UART_Print_U16_List(const uint16_t *src, size_t count)
{
    char line[16];
    for (size_t i = 0; i < count; i++)
    {
        int n = snprintf(line, sizeof(line), "%u\r\n", (unsigned)src[i]);
        if (n > 0)
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 50);
    }
}
*/
// ADC DMA yarım dolduğunda tetiklenir
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3)
    {
#if DEBUG_ASCII
        if (!dbg_gate) {                  // debug modunda seyrek yazdır
            dbg_gate = 1;
            UART_Print_ADC_Analysis((const uint16_t*)adc_buf, 32); // İlk yarının bir kısmını analiz et
        }
#else
        // Binary mod: ham veriyi DMA ile gönder
        if (uart_ready) {
            uart_ready = 0;
            // UART DMA bitene kadar bekle (çakışmayı önler)
            uint32_t timeout = 1000;
            while((huart2.gState != HAL_UART_STATE_READY) && (timeout--)) {
                HAL_Delay(1);
            }
            HAL_UART_Transmit_DMA(&huart2,
                                   (uint8_t*)adc_buf,
                                   (ADC_BUF_LEN/2) * sizeof(uint16_t));
        }
#endif
    }
}

// ADC DMA tamamen dolduğunda tetiklenir
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3)
    {
#if DEBUG_ASCII
        if (!dbg_gate) {
            dbg_gate = 1;
            UART_Print_ADC_Analysis((const uint16_t*)&adc_buf[ADC_BUF_LEN/2], 32);  // İkinci yarının bir kısmını analiz et
        }
#else
        // Binary mod: ham veriyi DMA ile gönder
        if (uart_ready) {
            uart_ready = 0;
            // UART DMA bitene kadar bekle (çakışmayı önler)
            uint32_t timeout = 1000;
            while((huart2.gState != HAL_UART_STATE_READY) && (timeout--)) {
                HAL_Delay(1);
            }
            HAL_UART_Transmit_DMA(&huart2,
                                   (uint8_t*)&adc_buf[ADC_BUF_LEN/2],
                                   (ADC_BUF_LEN/2) * sizeof(uint16_t));
        }
#endif
    }
}

// UART DMA tamamlandığında - tekrar gönderime hazır
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        uart_ready = 1;
}

// UART hata durumu
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_UART_AbortTransmit(huart);
        uart_ready = 1;
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Başlangıç mesajı
  char init_msg[] = "STM32 DAC+ADC System Starting...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)init_msg, strlen(init_msg), 1000);

  // 1) Sinüs dalga tablosunu hazırla
  Fill_SineWave();

  // 2) DAC CH1 (PA4) DMA + TIM7 TRGO ile hazırla
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                       (uint32_t*)dac_wave, SINE_LEN, DAC_ALIGN_12B_R) != HAL_OK) {
      Error_Handler();
  }

  // 3) ADC3 (PB1) kalibrasyon ve DMA hazırlığı
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK) {
      Error_Handler();
  }

  // 4) Timer'ları başlat - DAC ve ADC senkron olarak başlar
  if (HAL_TIM_Base_Start(&htim7) != HAL_OK) {   // TIM7 -> DAC trigger
      Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {   // TIM6 -> ADC trigger
      Error_Handler();
  }

  // Başlangıç tamamlandı mesajı
  char ready_msg[] = "System Ready! DAC: PA4, ADC: PB1\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)ready_msg, strlen(ready_msg), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if DEBUG_ASCII
    dbg_gate = 0;      // her seferinde debug yazdırılmasını sağla
    HAL_Delay(250);    // ~4 Hz yazdır (TeraTerm için uygun hız)
#else
    HAL_Delay(10);     // Binary modda daha hızlı çevrim
#endif
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 921600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PB1 -> ADC3_IN1 : Analog Input */
  GPIO_InitStruct.Pin  = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA4 -> DAC1_OUT1 : Analog Output */
  GPIO_InitStruct.Pin  = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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

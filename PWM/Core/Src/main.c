/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PWM kontrolü - WebSocket ile
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

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
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t pwm_deger = 50;           // PWM değeri başlangıç %50
uint8_t gelen_karakter;           // UART'tan gelen tek karakter
char komut_buffer[20];            // Komut yazacağımız yer
uint8_t buffer_sayac = 0;         // Kaçıncı karakterdeyiz
char gonderilecek_mesaj[100];     // Gönderilecek mesaj
uint32_t pwm_frekans = 1000;      // PWM frekansı (Hz)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
// PWM değerini ayarlayan fonksiyon
void PWM_Ayarla(uint8_t yuzde)
{
    if (yuzde > 100) yuzde = 100;  // Maksimum %100 olsun

    uint32_t timer_max = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t pwm_degeri = ((timer_max + 1) * yuzde) / 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_degeri);
    pwm_deger = yuzde;
}

// PWM frekansını ayarla
void PWM_Frekans_Ayarla(uint32_t frekans_hz)
{
    if (frekans_hz < 1) frekans_hz = 1;
    if (frekans_hz > 100000) frekans_hz = 100000;  // Max 100kHz

    // 170MHz sistem saati
    // Prescaler = 170, Timer Clock = 1MHz
    // Period = (1MHz / istenen_frekans) - 1
    uint32_t period = (1000000 / frekans_hz) - 1;

    if (period < 10) period = 10;        // Minimum değer
    if (period > 65535) period = 65535;   // 16-bit timer limiti

    __HAL_TIM_SET_AUTORELOAD(&htim3, period);
    pwm_frekans = frekans_hz;

    // Mevcut duty cycle'ı koru
    PWM_Ayarla(pwm_deger);
}

// String'den sayı çıkarma
int StringdenSayiyaCevir(char *str)
{
    int sonuc = 0;
    while (*str >= '0' && *str <= '9') {
        sonuc = sonuc * 10 + (*str - '0');
        str++;
    }
    return sonuc;
}

// WebSocket için JSON formatında veri gönder
void WebSocket_Veri_Gonder(void)
{
    uint32_t timer_max = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr_deger = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);

    // JSON format: ("duty":50,"freq":1000,"arr":999,"ccr":500)
    snprintf(gonderilecek_mesaj, sizeof(gonderilecek_mesaj),
             "{\"duty\":%d,\"freq\":%lu,\"arr\":%lu,\"ccr\":%lu}\r\n",
             pwm_deger, pwm_frekans, timer_max, ccr_deger);
    HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
}

// UART'tan komut geldiğinde
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Enter'a bastı mı?
        if (gelen_karakter == '\n' || gelen_karakter == '\r') {
            if (buffer_sayac > 0) {
                komut_buffer[buffer_sayac] = '\0';  // String bitir

                // Komut analizi
                if (komut_buffer[0] == 'd' || komut_buffer[0] == 'D') {  // Duty cycle
                    int yeni_duty = StringdenSayiyaCevir(&komut_buffer[1]);
                    if (yeni_duty >= 0 && yeni_duty <= 100) {
                        PWM_Ayarla(yeni_duty);
                        sprintf(gonderilecek_mesaj, "Duty cycle: %%%d\r\n> ", pwm_deger);
                        HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
                    }
                    else {
                        sprintf(gonderilecek_mesaj, "HATA! Duty cycle 0-100 arasinda olmali!\r\n> ");
                        HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
                    }
                }
                else if (komut_buffer[0] == 'f' || komut_buffer[0] == 'F') {  // Frekans
                    int yeni_frekans = StringdenSayiyaCevir(&komut_buffer[1]);
                    if (yeni_frekans >= 1 && yeni_frekans <= 100000) {
                        PWM_Frekans_Ayarla(yeni_frekans);
                        sprintf(gonderilecek_mesaj, "Frekans: %lu Hz\r\n> ", pwm_frekans);
                        HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
                    }
                    else {
                        sprintf(gonderilecek_mesaj, "HATA! Frekans 1-100000 Hz arasinda olmali!\r\n> ");
                        HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
                    }
                }
                else if (komut_buffer[0] == 's' || komut_buffer[0] == 'S') {  // Status
                    WebSocket_Veri_Gonder();
                    sprintf(gonderilecek_mesaj, "> ");
                    HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
                }
                else {
                    sprintf(gonderilecek_mesaj, "Bilinmeyen komut! d50, f1000, s kullan\r\n> ");
                    HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_mesaj, strlen(gonderilecek_mesaj), 100);
                }

                buffer_sayac = 0;
            }
        }
        // Silme tuşu
        else if (gelen_karakter == 8 || gelen_karakter == 127) {
            if (buffer_sayac > 0) {
                buffer_sayac--;
                char sil[] = "\b \b";
                HAL_UART_Transmit(&huart2, (uint8_t*)sil, 3, 10);
            }
        }
        // Normal karakter
        else if (gelen_karakter >= ' ') {
            if (buffer_sayac < sizeof(komut_buffer) - 1) {
                komut_buffer[buffer_sayac] = gelen_karakter;
                buffer_sayac++;
                HAL_UART_Transmit(&huart2, &gelen_karakter, 1, 10);
            }
        }

        // Bir sonraki karakteri dinle
        HAL_UART_Receive_IT(&huart2, &gelen_karakter, 1);
    }
}
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // PWM'i başlat
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  PWM_Frekans_Ayarla(pwm_frekans);  // İlk frekansı ayarla
  PWM_Ayarla(pwm_deger);            // İlk duty cycle'ı ayarla

  // UART'ı dinlemeye başla
  HAL_UART_Receive_IT(&huart2, &gelen_karakter, 1);

  // Başlangıç mesajı
  HAL_Delay(100);
  char baslangic[] = "\r\n=== PWM WebSocket Kontrolu ===\r\n"
                     "Komutlar:\r\n"
                     "  d50   : Duty cycle %50\r\n"
                     "  f1000 : Frekans 1000Hz\r\n"
                     "  s     : Status gonder\r\n"
                     "PWM Cikisi: PA7\r\n> ";
  HAL_UART_Transmit(&huart2, (uint8_t*)baslangic, strlen(baslangic), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t son_gonderim = 0;

  while (1)
  {
    // Her 100ms'de bir otomatik status gönder
    if (HAL_GetTick() - son_gonderim > 100) {
        WebSocket_Veri_Gonder();
        son_gonderim = HAL_GetTick();
    }

    HAL_Delay(10);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitStruct structure.
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170-1;     // 170MHz / 170 = 1MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;          // 1MHz / 1000 = 1kHz PWM
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
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

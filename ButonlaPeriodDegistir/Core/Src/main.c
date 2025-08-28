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

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint32_t led_blink_period_ms;      // LED yanıp sönme periyodu
    uint32_t led_blink_timestamp_ms;   // Son LED toggle zamanı
} LedTaskVars;

/* USER CODE BEGIN PV */
static LedTaskVars led_task = {
    .led_blink_period_ms    = 250,
    .led_blink_timestamp_ms = 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
// LED blink işlevi
static void led_blink_task(void) {
    uint32_t current_time = HAL_GetTick();
    if ((current_time - led_task.led_blink_timestamp_ms) >= led_task.led_blink_period_ms) {
        led_task.led_blink_timestamp_ms = current_time;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief  EXTI IRQ callback
  * @param  GPIO_Pin: interrupt kaynağı pin
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_irq_ts = 0;
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_13)
    {
        /* Debounce: 40 ms içinde tekrar geliyorsa yok say */
        if (now - last_irq_ts < 40) return;
        last_irq_ts = now;

        /* Butona her basışta periyodu 250 ms artır, 2000 ms geçince 250 ms'e resetle */
        led_task.led_blink_period_ms += 250;
        if (led_task.led_blink_period_ms > 2000) {
            led_task.led_blink_period_ms = 250;
        }
    }
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    led_blink_task();
    /* user_button polling işlevi artık gerek yok */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};

  /* Internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* HSI osilatörünü başlat */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* CPU, AHB ve APB clock ayarları */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   |
                                     RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1  |
                                     RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO portlarının clock'unu etkinleştir */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* LED (PA5) başlangıç değeri */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /* PC13: USER BUTTON, External Interrupt Falling Edge, pull-up */
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA5: LED çıkış */
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI15_10 IRQ ayarları */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  Bu fonksiyon hata oluştuğunda çalışır
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
/**
  * @brief  assert_param hatası durumunda raporlama
  * @param  file: kaynak dosya ismi
  * @param  line: satır numarası
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Kullanıcı raporlama ekleyebilir */
}
#endif /* USE_FULL_ASSERT */

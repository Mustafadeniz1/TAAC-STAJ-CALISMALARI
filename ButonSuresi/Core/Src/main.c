/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"            /* TIM2 handler, MX_TIM2_Init() */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t led_blink_period_ms;      /* LED yanıp sönme periyodu (ms) */
    uint32_t led_blink_timestamp_ms;   /* Son LED toggle zamanı (ms) */
} LedTaskVars;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static LedTaskVars led_task = {
    .led_blink_period_ms    = 250,
    .led_blink_timestamp_ms = 0
};
static volatile uint32_t edge_start;     /* Düşen kenarda başlama sayacı */
static volatile uint32_t edge_end;       /* Yükselen kenarda bitiş sayacı */
static volatile uint32_t pulse_width_us; /* Ölçülen pulse width (µs) */
static volatile uint8_t  measuring = 0;  /* Ölçüm başladı mı flag */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void       SystemClock_Config(void);
static void MX_GPIO_Init(void);
void       Error_Handler(void);
/* USER CODE BEGIN PFP */
/**
  * @brief  LED toggle işlevi: mevcut periyoda göre toggle eder
  */
static void led_blink_task(void)
{
    uint32_t current_tick = HAL_GetTick();  /* ms cinsinden zaman */
    if ((current_tick - led_task.led_blink_timestamp_ms) >= led_task.led_blink_period_ms)
    {
        led_task.led_blink_timestamp_ms = current_tick;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief  EXTI kesme callback: PC13 pinindeki interrupt buraya düşer
  *         Falling edge: ölçüme başla (measuring=1)
  *         Rising  edge: measuring==1 ise ölçümü bitir ve süreyi hesapla
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_irq_time_ms = 0;  /* Debounce için */
    uint32_t current_time = HAL_GetTick(); /* ms cinsinden zaman */
    if (GPIO_Pin != GPIO_PIN_13) return;
    /* Debounce: 40 ms içinde tekrar geliyorsa yok say */
    if ((current_time - last_irq_time_ms) < 40) return;
    last_irq_time_ms = current_time;
    /* Sayaçtan güncel değeri oku */
    uint32_t t = __HAL_TIM_GET_COUNTER(&htim2);
    /* FALLING edge: butona basıldı */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        measuring  = 1;
        edge_start = t;
    }
    /* RISING edge: measurement varsa hesapla */
    else if (measuring)
    {
        edge_end = t;
        if (edge_end >= edge_start)
            pulse_width_us = edge_end - edge_start;
        else
            pulse_width_us = (0xFFFFFFFF - edge_start) + edge_end + 1;
        measuring = 0;
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
    HAL_Init();                                   /* Reset, Flash, Systick */
    __HAL_RCC_SYSCFG_CLK_ENABLE();                /* EXTI mapping için */
    SystemClock_Config();                         /* Saat konfigürasyonu */
    MX_GPIO_Init();                               /* PA5 LED, PC13 EXTI */
    MX_TIM2_Init();                               /* TIM2: 1 MHz free-run */
    /* USER CODE BEGIN 2 */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)      /* TIM2 başlat */
    {
        Error_Handler();                          /* Hata durumunda */
    }
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        led_blink_task();                         /* LED blink */
        /* pulse_width_us: basılı kalma süresi (µs) */
    }
    /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                       |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PC13: USER button, rising+falling, no pull */
    GPIO_InitStruct.Pin  = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    /* PA5: LED output */
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* EXTI15_10 interrupt init */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Reports the name of the source file and the source line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

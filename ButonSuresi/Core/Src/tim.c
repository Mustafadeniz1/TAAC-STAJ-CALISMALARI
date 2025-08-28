/* tim.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Pc
 */

#include "tim.h"

TIM_HandleTypeDef htim2;  /* 32-bit free-run timer handle */

void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* Enable TIM2 peripheral clock */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Base timer configuration: 1 MHz counter, full-scale 32-bit */
    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = (SystemCoreClock / 1000000) - 1; /* 1 MHz */
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure clock source to internal */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Master configuration: no TRGO, no master/slave */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

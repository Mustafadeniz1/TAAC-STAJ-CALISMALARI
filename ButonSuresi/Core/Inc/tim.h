/* tim.h
 *
 *  Created on: Aug 6, 2025
 *      Author: Pc
 */
#ifndef INC_TIM_H_
#define INC_TIM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32g4xx_hal_tim.h"

/* TIM2 handler declared in tim.c */
extern TIM_HandleTypeDef htim2;

/**
  * @brief  Configure TIM2 as a 32-bit, free-running counter @ 1 MHz (1 µs resolution)
  */
void MX_TIM2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIM_H_ */

#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

#define PWM_GPIO_PIN				GPIO_Pin_7
#define PWM_GPIO_PORT				GPIOA
#define PWM_GPIO_SOURCE			GPIO_PinSource7
#define PWM_GPIO_RCC_AHB1Periph			RCC_AHB1Periph_GPIOA

#define PWM_TIM4_GPIO_PIN					GPIO_Pin_6
#define PWM_TIM4_GPIO_PORT				GPIOB
#define PWM_TIM4_GPIO_SOURCE			GPIO_PinSource6
#define PWM_TIM4_GPIO_RCC_AHB1Periph			RCC_AHB1Periph_GPIOB

void TIM3_PWM_Init(u32 arr,u32 psc);

void TIM4_PWM_Init(u32 arr,u32 psc);

#endif

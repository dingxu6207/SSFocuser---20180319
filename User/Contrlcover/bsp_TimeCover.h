#ifndef __BSP_TIMECOVER_H
#define __BSP_TIMECOVER_H

#include "stm32f10x.h"

#define            COVER_TIM                   TIM2
#define            COVER_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            COVER_TIM_CLK               RCC_APB1Periph_TIM2
#define            COVER_TIM_Period            (200-1)    
#define            COVER_TIM_Prescaler         (7200-1)
#define            COVER_TIM_IRQ               TIM2_IRQn
#define            COVER_TIM_IRQHandler        TIM2_IRQHandler

// TIM2 输出比较通道
#define COVER_TIM_CH2_GPIO_CLK 		RCC_APB2Periph_GPIOB
#define COVER_TIM_CH2_PORT 			GPIOB
#define COVER_TIM_CH2_PIN 			GPIO_Pin_3


void Cover_TIM_Init(void);
void ControlCover(FunctionalState NewState);
void SetSpeedCover(u16 uSpeedCoverOne);
//void Timer2_Configuration(void);

#endif	/* __BSP_TIMEBASE_H */

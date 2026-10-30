#ifndef BSP_PWM_H
#define BSP_PWM_H
#include <stdint.h>
#include "stm32f10x.h"
#define PWM_NUM    4
typedef struct
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    uint32_t GPIO_Clock;
    uint32_t TIM_Clock;
    TIM_TypeDef* TIMx;
}PWM_Channel_t;

void BSP_PWM_Init(void);
void BSP_PWM_SetCompare1(uint16_t Compare);
void BSP_PWM_SetCompare2(uint16_t Compare);
void BSP_PWM_SetCompare3(uint16_t Compare);
void BSP_PWM_SetCompare4(uint16_t Compare);
#endif
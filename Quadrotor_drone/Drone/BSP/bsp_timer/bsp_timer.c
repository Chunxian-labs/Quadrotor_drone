#include "stm32f10x.h"
#include "bsp_timer.h"

void System_timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65536-1;
    // TIM3 clock = 72 MHz: 72 / 72 = 1 MHz, one count per microsecond.
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

    // Delay uses the counter directly, so no update interrupt is needed.
    TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
    TIM_ClearFlag(TIM3,TIM_FLAG_Update);
    TIM_SetCounter(TIM3,0);

    TIM_Cmd(TIM3,ENABLE);
}
void System_Delay_us(uint32_t us)
{
    /* Use short chunks so each measurement fits in the 16-bit counter. */
    while (us != 0U)
    {
        uint16_t ticks = (us > 1000U) ? 1000U : (uint16_t)us;
        uint16_t start = TIM_GetCounter(TIM3);
        /* Unsigned subtraction also handles a counter wrap at 65535. */
        while ((uint16_t)(TIM_GetCounter(TIM3) - start) < ticks);
        us -= ticks;
    }
}

void System_Delay_ms(uint32_t ms)
{
    /* Avoid overflow from multiplying a large millisecond value by 1000. */
    while (ms != 0U)
    {
        System_Delay_us(1000U);
        --ms;
    }
}


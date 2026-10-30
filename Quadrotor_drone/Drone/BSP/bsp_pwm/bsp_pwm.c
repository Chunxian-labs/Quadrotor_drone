#include "stm32f10x.h"
#include "bsp_pwm.h"

static const PWM_Channel_t PWM_Channel_Cfg_talbe[PWM_NUM] = 
{
    {GPIOA,GPIO_Pin_0,RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM2,TIM2},
    {GPIOA,GPIO_Pin_1,RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM2,TIM2},
    {GPIOA,GPIO_Pin_2,RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM2,TIM2},
    {GPIOA,GPIO_Pin_3,RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM2,TIM2},
};

void BSP_PWM_Init(void)
{
    for(uint8_t i=0;i<PWM_NUM;i++)
    {
        RCC_APB2PeriphClockCmd(PWM_Channel_Cfg_talbe[i].GPIO_Clock,ENABLE);
        if(PWM_Channel_Cfg_talbe[i].TIMx == TIM1)
        {
            RCC_APB2PeriphClockCmd(PWM_Channel_Cfg_talbe[i].TIM_Clock,ENABLE);
        }
        else
        {
            RCC_APB1PeriphClockCmd(PWM_Channel_Cfg_talbe[i].TIM_Clock,ENABLE);
        }
    }
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    for(uint8_t i=0;i<PWM_NUM;i++)
    {
        GPIO_InitStructure.GPIO_Pin = PWM_Channel_Cfg_talbe[i].GPIO_Pin;
        GPIO_Init(PWM_Channel_Cfg_talbe[i].GPIOx,&GPIO_InitStructure);
    }
    TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_Prescaler = 36-1;
    TIM_TimeBaseStructure.TIM_Period = 100-1;//PWM周期为50us
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM2,&TIM_OCInitStructure);
    TIM_OC2Init(TIM2,&TIM_OCInitStructure);
    TIM_OC3Init(TIM2,&TIM_OCInitStructure);
    TIM_OC4Init(TIM2,&TIM_OCInitStructure);

    TIM_Cmd(TIM2,ENABLE);
}
void BSP_PWM_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2,Compare);
}
void BSP_PWM_SetCompare2(uint16_t Compare)
{
    TIM_SetCompare2(TIM2,Compare);
}
void BSP_PWM_SetCompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM2,Compare);
}
void BSP_PWM_SetCompare4(uint16_t Compare)
{
    TIM_SetCompare4(TIM2,Compare);
}  

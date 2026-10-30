#include "AD.h"
#include "stm32f10x.h"
typedef struct
{
    GPIO_TypeDef* GPIO_PORT;
    uint16_t      GPIO_PIN;
    uint8_t       ADC_CHANNEL;
}AD_cfg;
const static AD_cfg AD_cfg_table[AD_CHANNEL_NUM] =
{
    {GPIOA, GPIO_Pin_0, ADC_Channel_0},
    {GPIOA, GPIO_Pin_1, ADC_Channel_1},
    {GPIOA, GPIO_Pin_2, ADC_Channel_2},
    {GPIOA, GPIO_Pin_3, ADC_Channel_3}
};
static AD_cfg AD[AD_CHANNEL_NUM];
static float AD_LPF[AD_CHANNEL_NUM]; //存储通过低通滤波处理后的ADC值
uint16_t AD_Value[AD_CHANNEL_NUM];
void BSP_AD_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);   //开启ADC时钟

    RCC_ADCCLKConfig(RCC_PCLK2_Div6); //主控芯片主频72MHz,除以6=12MHz,ADC最大时钟频率为14MHz
    for(uint8_t i=0;i<AD_CHANNEL_NUM;i++)
    {
        AD[i] = AD_cfg_table[i];
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_Pin = AD[i].GPIO_PIN;
        GPIO_Init(AD[i].GPIO_PORT, &GPIO_InitStruct);
    }
    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;//ADC1单独工作，不与ADC2协同
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//右对齐，8位有效位
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//外部触发转换，软件触发
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;//单次转换模式
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;//单通道转换
    ADC_InitStruct.ADC_NbrOfChannel = 1;//转换1个通道
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);
}
void BSP_AD_Scan(void)
{
    for(uint8_t i=0;i<AD_CHANNEL_NUM;i++)
    {
        //选用ADC1,转换通道为channel,采样时间为55个时钟周期
        ADC_RegularChannelConfig(ADC1, AD[i].ADC_CHANNEL, 1,ADC_SampleTime_55Cycles5);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        uint16_t raw_data = ADC_GetConversionValue(ADC1);
        /* 一阶低通：alpha 越小越平滑，但越滞后 */
        AD_LPF[i] += 0.3f * (raw_data - AD_LPF[i]);//新值=旧值+0.3×(本次采样-旧值)
        AD_Value[i] = (uint16_t)AD_LPF[i];
    }
}

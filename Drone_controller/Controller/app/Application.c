#include "Application.h"
void Application_Init(void)
{
    BSP_TIM1_Init();
    BSP_AD_Init();
    BSP_Button_Init();
}
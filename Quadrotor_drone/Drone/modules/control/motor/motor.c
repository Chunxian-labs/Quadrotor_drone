#include "motor.h"
#include "bsp_pwm.h"
#include <math.h>
/* 最近一次下发的四路整数PWM是否全部为0 */
static uint8_t OutputAllZero = 1U;
static uint16_t Motor_Limit(float value)
{
    if(!isfinite(value) || value  <= 0.0f)
    {
        return 0;
    }
    if(value >= 100.0f)
    {
        return 100;
    }
    return (uint16_t)value;
}

void Motor_Init(void)
{
    BSP_PWM_Init();
    Motor_Stop();
}
void Motor_Stop(void)
{
    BSP_PWM_SetCompare1(0);
    BSP_PWM_SetCompare2(0);
    BSP_PWM_SetCompare3(0);
    BSP_PWM_SetCompare4(0);
    OutputAllZero = 1U;
}

void Motor_SetOutput(float a1,float a2,float a3,float a4)
{
    uint16_t pwm1 = Motor_Limit(a1);
    uint16_t pwm2 = Motor_Limit(a2);
    uint16_t pwm3 = Motor_Limit(a3);
    uint16_t pwm4 = Motor_Limit(a4);

    BSP_PWM_SetCompare1(pwm1);
    BSP_PWM_SetCompare2(pwm2);
    BSP_PWM_SetCompare3(pwm3);
    BSP_PWM_SetCompare4(pwm4);

    OutputAllZero =
        (pwm1 == 0U && pwm2 == 0U &&
         pwm3 == 0U && pwm4 == 0U);
}
void Motor_Mix(float throttle,float roll,float pitch,float yaw,MotorOutput_t *output)
{
    /* 混合输出 
        m1,m2,m3,m4依次为左上，右上，左下，右下电机
        throttle为基础油门，roll，pitch，yaw分别为横滚，俯仰，偏航对应的油门变化量
        同一对角（同旋向）的两个电机 yaw 符号相等，它们产生的反扭矩同向、等大，叠加后形成偏航力矩；
        而另一对对角 yaw 符号相反，产生反向反扭矩。悬停时两对反扭矩抵消使飞机不转；偏航时两对差值产生净扭矩使机头转动。
    */
    output->m1 = throttle + roll + pitch + yaw;
    output->m2 = throttle - roll + pitch - yaw;
    output->m3 = throttle + roll - pitch - yaw;
    output->m4 = throttle - roll - pitch + yaw;
}

/* 返回最近下发的PWM是否全部为0，不代表检测到电机实际停转 */
uint8_t Motor_IsOutputAllZero(void)
{
    return OutputAllZero;
}
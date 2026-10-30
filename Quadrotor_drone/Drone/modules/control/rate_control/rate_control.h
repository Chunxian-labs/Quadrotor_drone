#ifndef _RATE_CONTROL_H
#define _RATE_CONTROL_H

#include "pid.h"
/* 三轴角速度控制器 */
typedef struct
{
    PID_t Roll;
    PID_t Pitch;
    PID_t Yaw;
} RateControl_t;
/* 三轴目标或测量角速度，单位 °/s */
typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
}RateVector_t;

void RateControl_Init(RateControl_t *RateControl);
/* 清除三轴PID运行状态，保留控制参数 */
void RateControl_Reset(RateControl_t *control);
void RateControl_Update(RateControl_t *RateControl, const RateVector_t *target,const RateVector_t *actual,float dt);

#endif
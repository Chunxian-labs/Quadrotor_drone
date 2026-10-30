#ifndef __ANGLE_CONTROL_H__
#define __ANGLE_CONTROL_H__
#include "pid.h"
#include "attitude.h"
#include "rate_control.h"
/* 三轴角度外环 */
typedef struct
{
    PID_t Roll;
    PID_t Pitch;
    PID_t Yaw;
} AngleControl_t;

/* 三轴目标角度，单位：度 */
typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
} AngleTarget_t;

void AngleControl_Init(AngleControl_t *AngleControl);
void AngleControl_Reset(AngleControl_t *AngleControl);
/*
 * 输入目标角度与实际角度。
 * 输出三轴目标角速度，单位：°/s。
 * 实际角度中的横滚、俯仰微调由应用层加入。
 */
void AngleControl_Update(AngleControl_t *AngleControl,const AngleTarget_t *Target,const AttitudeEuler *Actual,
                            RateVector_t *Target_rate,float dt);
#endif
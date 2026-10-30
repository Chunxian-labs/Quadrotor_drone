#include "vertical_control.h"
//状态重置
void VerticalControl_Reset(VerticalControl_t *control)
{
    PID_Init(&control->velocity);
}
//上电初始化
void VerticalControl_Init(VerticalControl_t *control)
{
    control->velocity = (PID_t){
        .kp = 1.50f,
        .ki = 2.00f,
        .kd = 0.10f,
        .OutMax = 100.0f,
        .OutMin = -100.0f
    };
    VerticalControl_Reset(control);
}
//垂直速度控制
void VerticalControl_Update(VerticalControl_t *control, float target_cm_s, float actual_cm_s, float dt_s)
{
    control->velocity.Target = target_cm_s;
    control->velocity.Actual = actual_cm_s;

    PID_Update(&control->velocity,dt_s);
}

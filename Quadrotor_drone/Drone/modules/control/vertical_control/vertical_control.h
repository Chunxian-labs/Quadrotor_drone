#ifndef VERTICAL_CONTROL_H
#define VERTICAL_CONTROL_H

#include "pid.h"

/* ==================== 垂直速度控制器 ==================== */

typedef struct
{
    PID_t velocity; /* 垂直速度PID：输入cm/s，输出集体油门 */
} VerticalControl_t;

/* 设置参数并清除运行状态，上电时调用 */
void VerticalControl_Init(VerticalControl_t *control);

/* 清除PID目标、实际值、误差及输出，保留参数 */
void VerticalControl_Reset(VerticalControl_t *control);

/*
 * 更新垂直速度PID。
 * target_cm_s：目标垂直速度。
 * actual_cm_s：三阶互补估计输出的垂直速度。
 * dt_s：调用周期，应用层传入0.008秒。
 *
 * 结果保存在control->velocity.Out，直接作为集体油门，
 * 不额外叠加悬停油门。
 */
void VerticalControl_Update(
    VerticalControl_t *control,
    float target_cm_s,
    float actual_cm_s,
    float dt_s);

#endif
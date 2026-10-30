#include "rate_control.h"

void RateControl_Reset(RateControl_t *control)
{
    PID_Init(&control->Roll);
    PID_Init(&control->Pitch);
    PID_Init(&control->Yaw);
}
// 初始化三轴角速度控制器,给PID各个成员赋初始值
void RateControl_Init(RateControl_t *RateControl)
{
    /* 横滚角速度PID */
    RateControl -> Roll = (PID_t){
        .kp = 0.15f,
        .ki = 0.10f,
        .kd = 0.0f,

        .OutMax = 100.0f,
        .OutMin = -100.0f
    };
     /* 俯仰角速度PID*/
    RateControl -> Pitch = (PID_t){
        .kp = 0.15f,
        .ki = 0.10f,
        .kd = 0.0f,

        .OutMax = 100.0f,
        .OutMin = -100.0f
    };    
     /* 偏航角速度PID */
    RateControl -> Yaw = (PID_t){
        .kp = 0.30f,
        .ki = 0.0f,
        .kd = 0.0f,

        .OutMax = 100.0f,
        .OutMin = -100.0f
    };
    
    RateControl_Reset(RateControl);
}

void RateControl_Update(RateControl_t *RateControl, const RateVector_t *target,const RateVector_t *actual,float dt)
{
    /* 横滚角速度 */
    RateControl->Roll.Target = target->Roll;
    RateControl->Roll.Actual = actual->Roll;
    /* 俯仰角速度 */
    RateControl->Pitch.Target = target->Pitch;
    RateControl->Pitch.Actual = actual->Pitch;
    /* 偏航角速度 */
    RateControl->Yaw.Target = target->Yaw;
    RateControl->Yaw.Actual = actual->Yaw;

    /*
    调用后，三轴修正量分别保存在：
    control->Roll.Out
    control->Pitch.Out
    control->Yaw.Out
    */
    PID_Update(&RateControl->Roll, dt);
    PID_Update(&RateControl->Pitch, dt);
    PID_Update(&RateControl->Yaw, dt);
}
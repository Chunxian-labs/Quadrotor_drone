#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid)
{
    /* 清除状态，保留参数与限幅配置 */
    pid->Target = 0.0f;
    pid->Actual = 0.0f;
    pid->Out = 0.0f;

    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->ErrorIntegral = 0.0f;
}

void PID_Update(PID_t *pid,float dt)
{
    /* 无效输入时清除运行状态和输出 */
    if(!isfinite(dt) || dt <= 0.0f || !isfinite(pid->Target) || !isfinite(pid->Actual))
    {
        PID_Init(pid);
        return;
    }
    /* 先保存上次误差，再计算当前误差 */
    pid->Error1 = pid->Error0;
    pid->Error0 = pid->Target - pid->Actual;


    /*需要对I项单独处理*/
    if(pid->ki != 0.0f)
    {
        pid->ErrorIntegral += pid->Error0 * dt;
    }
    else
    {
        pid->ErrorIntegral = 0.0f;
    }    
    float p_out = pid->kp * pid->Error0;
    float i_out = pid ->ki * pid->ErrorIntegral;
    float d_out = pid->kd * (pid->Error0 - pid->Error1) / dt;
    /* 计算总输出 */
    pid->Out = p_out + i_out + d_out;
    if(!isfinite(pid->Out))
    {
        PID_Init(pid);
        return;
    }
    /* 限制总输出在指定范围内 */
    if(pid->Out > pid->OutMax)
    {
        pid->Out = pid->OutMax;
    }
    else if(pid->Out < pid->OutMin)
    {
        pid->Out = pid->OutMin;
    }
}
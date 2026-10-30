#ifndef _PID_H
#define _PID_H
#include <stdint.h>
typedef struct
{
    float Target;       /* 目标值 */
    float Actual;       /* 实际值 */
    float Out;          /* 输出值 */
    /* PID 参数 */
    float kp;
    float ki;
    float kd;

    float Error0;         /* 当前误差 */
    float Error1;         /* 上次误差 */
    float ErrorIntegral;  /* 误差积分项，未乘 Ki */
    
    float IntegralMax; /* 暂供旧模块引用，原厂PID计算不使用 */
    float OutMax;       /* 总输出上限 */
    float OutMin;       /* 总输出下限 */
}PID_t;

void PID_Init(PID_t *pid);
void PID_Update(PID_t *pid,float dt);
#endif
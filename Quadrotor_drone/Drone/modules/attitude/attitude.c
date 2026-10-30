#include "attitude.h"
#include <math.h>
#define RAD_TO_DEG          57.2957795f      // 弧度转角度
#define DEG_TO_RAD          0.01745329252f   // 角度转弧度
#define ATTITUDE_KP         0.2f
#define ATTITUDE_KI         0.1f
/* 积分反馈，单位 rad/s */
static float IntegralX = 0.0f;
static float IntegralY = 0.0f;
static float IntegralZ = 0.0f;
/* 检查三轴数据是否可用 */
static uint8_t Vector_IsFinite(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z);
}
/* 姿态初始化,先用三轴的加速度计算出上电后的四元数*/
uint8_t Attitude_Init(AttitudeQuat *Quat,float ax,float ay,float az)
{
    if(!Vector_IsFinite(ax,ay,az))
    {
        return 0;
    }
    float acc_norm_sq = ax * ax + ay * ay + az * az;   //用于后续归一化
    if((acc_norm_sq < 1e-6f) || !isfinite(acc_norm_sq))
    {
        return 0;
    }
    /* 通过加速度确定初始横滚和俯仰 */
    float roll = atan2f(ay,az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    /*
     * 初始化yaw为0。
     * 欧拉角转四元数公式可以简化成下面四项。
     * c = cosine，s = sine，r = roll，p = pitch
     */
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    float q0 = cr * cp;
    float q1 = sr * cp;
    float q2 = cr * sp;
    float q3 = -sr * sp;

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (!isfinite(norm) || norm < 0.000001f)
    {
        return 0;
    }

    Quat->q0 = q0 / norm;
    Quat->q1 = q1 / norm;
    Quat->q2 = q2 / norm;
    Quat->q3 = q3 / norm;

    /* 重新建立姿态时，清除旧的误差积分 */
    IntegralX = 0.0f;
    IntegralY = 0.0f;
    IntegralZ = 0.0f;

    return 1;
}
/* 六轴姿态融合 */
uint8_t Attitude_Update6Axis(AttitudeQuat *Quat,float gx,float gy,float gz,float ax,float ay,float az,float dt)
{
    if(!Vector_IsFinite(ax,ay,az) || !Vector_IsFinite(gx,gy,gz) || !isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }
    /* 加速度归一化 */
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (!isfinite(norm) || norm < 0.000001f)
    {
        return 0;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    /* 保存本轮开始时的四元数 */
    float q0 = Quat->q0;
    float q1 = Quat->q1;
    float q2 = Quat->q2;
    float q3 = Quat->q3;
    /* 由四元数提取理论重力方向 */
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = 1.0f - 2.0f * (q1*q1 + q2*q2);

    /* 实测方向 × 理论方向 */
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    float ex_int = IntegralX + ex * dt;
    float ey_int = IntegralY + ey * dt;
    float ez_int = IntegralZ + ez * dt;

    /* 角速度从°/s转为rad/s，并加入PI补偿 */
    gx = gx * DEG_TO_RAD
       + ATTITUDE_KP * ex + ATTITUDE_KI * ex_int;

    gy = gy * DEG_TO_RAD
       + ATTITUDE_KP * ey + ATTITUDE_KI * ey_int;

    gz = gz * DEG_TO_RAD
       + ATTITUDE_KP * ez + ATTITUDE_KI * ez_int;

    /* 四个新分量统一使用旧四元数计算 */
    float next_q0 =
        q0 + 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;

    float next_q1 =
        q1 + 0.5f * ( q0*gx - q3*gy + q2*gz) * dt;

    float next_q2 =
        q2 + 0.5f * ( q3*gx + q0*gy - q1*gz) * dt;

    float next_q3 =
        q3 + 0.5f * (-q2*gx + q1*gy + q0*gz) * dt;
    /* 每轮更新后归一化 */
    norm = sqrtf(
        next_q0*next_q0 + next_q1*next_q1 +
        next_q2*next_q2 + next_q3*next_q3);

    if (!isfinite(norm) || norm < 0.000001f)
    {
        return 0;
    }

    /* 计算成功后，才提交新状态 */
    Quat->q0 = next_q0 / norm;
    Quat->q1 = next_q1 / norm;
    Quat->q2 = next_q2 / norm;
    Quat->q3 = next_q3 / norm;
    /* 更新误差积分 */
    IntegralX = ex_int;
    IntegralY = ey_int;
    IntegralZ = ez_int;

    return 1;
}
/*
*Euler角度输出函数
*这个函数把内部用四元数存储的姿态，按 ZYX 顺序反解成横滚、俯仰、偏航三个欧拉角（单位度），供控制、显示、日志使用。
*/
void Attitude_QuatToEuler(const AttitudeQuat *Quat,AttitudeEuler *Euler)
{
    float q0 = Quat->q0;
    float q1 = Quat->q1;
    float q2 = Quat->q2;
    float q3 = Quat->q3;
    /* 四元数转欧拉角公式-横滚角 */
    Euler->roll = atan2f(2.0f * (q2*q3 + q0*q1), 1.0f - 2.0f * (q1*q1 + q2*q2)) * RAD_TO_DEG;
    /* 防止浮点舍入使asinf输入超出[-1, 1] */
    float sin_pitch = 2.0f * (q0*q2 - q1*q3);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
    /* 四元数转欧拉角公式-俯仰角 */
    Euler->pitch = asinf(sin_pitch) * RAD_TO_DEG;
    /* 四元数转欧拉角公式-偏航角 */
    Euler->yaw = atan2f(2.0f * (q1*q2 + q0*q3),1.0f - 2.0f * (q2*q2 + q3*q3)) * RAD_TO_DEG;
}

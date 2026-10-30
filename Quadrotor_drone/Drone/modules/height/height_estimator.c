#include "height_estimator.h"
#include <math.h>

#define DEG_TO_RAD          0.01745329252f   // 角度转弧度
#define HEIGHT_DT_S         0.008f
/* 状态初始化 */
void HeightEstimator_Init(HeightEstimator_t *state)
{
    *state = (HeightEstimator_t){0};
}

/* 垂直加速度计算 */
uint8_t HeightEstimator_CalculateVerticalAccel(float ax, float ay, float az, float roll_deg, float pitch_deg, float *a_vertical)
{
    if (a_vertical == 0 ||
        !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(roll_deg) || !isfinite(pitch_deg))
    {
        return 0;
    }

    float sin_roll = sinf(roll_deg * DEG_TO_RAD);
    float cos_roll = cosf(roll_deg * DEG_TO_RAD);
    float sin_pitch = sinf(pitch_deg * DEG_TO_RAD);
    float cos_pitch = cosf(pitch_deg * DEG_TO_RAD);
    /* 将加速度投影到竖直方向 */
    float vertical_accel_cm_s2 = -ax * sin_pitch + ay * cos_pitch * sin_roll + az * cos_pitch * cos_roll;
    /* 去掉1g，再转换为cm/s² */
    vertical_accel_cm_s2 = (vertical_accel_cm_s2 - 1.0f) * 9.8f * 100.0f;
    if (!isfinite(vertical_accel_cm_s2))
    {
        return 0;
    }

    *a_vertical = vertical_accel_cm_s2;
    return 1;
}

/* ==================== 三阶互补估计 ==================== */
uint8_t HeightEstimator_Update(HeightEstimator_t *state, float vertical_accel_cm_s2, float measured_height_cm)
{
    if(!isfinite(vertical_accel_cm_s2) || !isfinite(measured_height_cm))
    {
        return 0;
    }
     /* 互补滤波的新息（innovation）：观测高度减去当前估计高度 */
    float error = measured_height_cm - state->height_cm;
    /* 高度误差限幅，防止气压突变/野值把三个积分器同时带飞，限制单轮高度误差反馈幅度,单位cm */
    if (error > 10.0f)
    {
        error = 10.0f;
    }
    else if (error < -10.0f)
    {
        error = -10.0f;
    }

    /* 第 1 个积分环节：把新息积分进"加速度修正量"，0.01 是该路的反馈增益 */
    float vertical_acc_cor = state->accel_correction_cm_s2 + 0.01f * error;
    /* 用修正量补偿加速度：a_vertical 提供高频动态，vertical_acc_cor 负责长期零偏/慢变误差 */
    float vertical_acc = vertical_accel_cm_s2 + vertical_acc_cor;

    /* 第 2 个积分环节：加速度积分成速度（用本轮修正后的加速度） */
    float vertical_vel = state->vertical_velocity_cm_s + vertical_acc * HEIGHT_DT_S;
    /* 把新息直接注入速度，0.02 为该路增益：加快速度收敛、抑制积分漂移（互补的"观测纠偏"） */
    vertical_vel += 0.02f * error;

    /* 第 3 个积分环节：用本轮更新后的速度积分出高度（先更新速度再积分位置） */
    float vertical_pos = state->height_cm + vertical_vel * HEIGHT_DT_S;
    /* 把新息直接注入位置，0.04 为该路增益：保证高度长期锁定到观测值（互补的低频基准） */
    vertical_pos += 0.04f * error;

    if (!isfinite(vertical_acc_cor) ||
        !isfinite(vertical_acc) ||
        !isfinite(vertical_vel) ||
        !isfinite(vertical_pos))
    {
        return 0;
    }

    state->accel_correction_cm_s2 = vertical_acc_cor;
    state->corrected_accel_cm_s2 = vertical_acc;
    state->vertical_velocity_cm_s = vertical_vel;
    state->height_cm = vertical_pos;

    return 1;
}
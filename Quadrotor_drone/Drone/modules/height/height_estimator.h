#ifndef HEIGHT_ESTIMATOR_H
#define HEIGHT_ESTIMATOR_H

#include <stdint.h>

/*
 * 三阶互补估计状态。
 */
typedef struct
{
    float accel_correction_cm_s2;  /* 加速度修正量 */
    float corrected_accel_cm_s2;   /* 修正后的垂直加速度 */
    float vertical_velocity_cm_s;  /* 估计垂直速度 */
    float height_cm;               /* 估计高度 */
} HeightEstimator_t;

/* 清零估计状态 */
void HeightEstimator_Init(HeightEstimator_t *state);

/*
 * 计算去重力后的垂直加速度。
 * 输入加速度：g，水平静止时az约+1。
 * 输入姿态角：度。
 * 输出：cm/s²。
 * 成功返回1；失败返回0，输出不变。
 */
uint8_t HeightEstimator_CalculateVerticalAccel(
    float ax, float ay, float az,
    float roll_deg, float pitch_deg,
    float *a_vertical);

/*
 * 三阶互补估计，每8ms调用一次。
 * vertical_accel_cm_s2：公式得到的垂直加速度，cm/s²。
 * measured_height_cm：高度观测，cm。
 * 成功返回1；失败返回0，状态不变。
 */
uint8_t HeightEstimator_Update(
    HeightEstimator_t *state,
    float vertical_accel_cm_s2,
    float measured_height_cm);

#endif
#ifndef HEIGHT_OBSERVATION_H
#define HEIGHT_OBSERVATION_H

#include <stdint.h>
typedef struct
{
    float baro_origin_m;       /* 起飞基准海拔，m */
    float baro_relative_cm;    /* 气压相对高度，cm */

    float baro_correction_cm;  /* 激光估计的气压高度修正量，cm */
    float corrected_height_cm; /* 气压相对高度＋修正量，cm */

    uint8_t baro_reference_ready; /* 已建立气压基准 */
} HeightObservation_t;

/*
 * 气压转海拔。
 * 输入：补偿后的气压，Pa。
 * 输出：气压海拔，m，尚未减起飞基准。
 * 成功返回1；失败返回0，输出不变。
 */
uint8_t HeightObservation_PressureToAltitude(float pressure_pa, float *altitude_m);
/*
 * 输入：驱动报告有效的新激光斜距mm，姿态角度°。
 * 小于40mm的距离拒绝使用。
 * 输出：垂直离地高度cm，不减40mm，不以首帧归零。
 *
 * 前提：激光沿机体向下安装，地面近似水平。
 * 成功返回1；失败返回0，输出不变。
 */
uint8_t HeightObservation_LaserToHeight(uint16_t distance_mm, float roll_deg, float pitch_deg, float *height_cm);

/* 清除高度观测状态 */
void HeightObservation_Init(HeightObservation_t *state);

/*
 * 用一份有效气压海拔建立起飞基准。
 * 在地面静止、进入运行前的初始化流程中调用。
 * 成功返回1；失败返回0，状态不变。
 */
uint8_t HeightObservation_SetBaroReference(HeightObservation_t *state, float altitude_m);

/*
 * 输入有效的新气压海拔，更新相对高度，单位cm。
 * 成功返回1；失败返回0，状态不变。
 */
uint8_t HeightObservation_UpdateBaro(HeightObservation_t *state, float altitude_m);

/*
 * 用一份新的激光高度观测，慢慢更新气压修正量。
 *
 * laser_reference_cm：已与气压起飞高度参考对齐的激光高度，cm。
 * dt_s：连续有效校正观测之间的间隔，秒。
 *
 * 调用者负责：
 * 新帧、测距状态、40mm下限、气压数据新鲜度及时间管理。
 * 激光中断后恢复时，重新建立计时，不把失效时长计入dt。
 *
 * 成功返回1；失败返回0，状态不变。
 */
uint8_t HeightObservation_CorrectBaro(HeightObservation_t *state, float laser_reference_cm, float dt_s);

#endif
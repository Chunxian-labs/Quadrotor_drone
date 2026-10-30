#include "height_observation.h"
#include <math.h>
#define DEG_TO_RAD            0.01745329252f   // 角度转弧度
#define BARO_CORRECTION_TAU_S 10.0f            // 慢变偏差校正时间常数
void HeightObservation_Init(HeightObservation_t *state)
{
    *state = (HeightObservation_t){0};
}
uint8_t HeightObservation_PressureToAltitude(float pressure_pa, float *altitude_m)
{
    if(altitude_m == 0 || !isfinite(pressure_pa) || pressure_pa <= 0.0f)
    {
        return 0;
    }
    float altitude = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 1.0f / 5.255f));
    if(!isfinite(altitude))
    {
        return 0;
    }
    *altitude_m = altitude;
    return 1;
}

uint8_t HeightObservation_LaserToHeight(uint16_t distance_mm, float roll_deg, float pitch_deg, float *height_cm)
{
    if(height_cm == 0 || distance_mm < 40 || !isfinite(roll_deg) || !isfinite(pitch_deg))
    {
        return 0;
    }
    if(fabsf(roll_deg) >= 90.0f || fabsf(pitch_deg) >= 90.0f)
    {
        return 0;
    }
    /*
     * 激光斜距投影到竖直方向。
     * mm除以10转换为cm。
     */
    float height = distance_mm / 10.0f
                    * cosf(roll_deg * DEG_TO_RAD)
                    * cosf(pitch_deg * DEG_TO_RAD);
    if(!isfinite(height))
    {
        return 0;
    }
    *height_cm = height;
    return 1;
}
uint8_t HeightObservation_SetBaroReference(HeightObservation_t *state, float altitude_m)
{
    if(!isfinite(altitude_m))
    {
        return 0;
    }
    state->baro_origin_m = altitude_m;
    state->baro_relative_cm = 0.0f;
    /* 基准改变后，旧修正量不能继续沿用 */
    state->baro_correction_cm = 0.0f;
    state->corrected_height_cm = 0.0f;
    state->baro_reference_ready = 1;
    return 1;
}

uint8_t HeightObservation_UpdateBaro(HeightObservation_t *state, float altitude_m)
{
    if(!state->baro_reference_ready || !isfinite(altitude_m))
    {
        return 0;
    }
    /* 当前海拔减起飞基准，再由m转换成cm */
    float relative_cm = (altitude_m - state->baro_origin_m) * 100.0f;
    /* 应用最近一次有效激光校正留下的修正量 */
    float corrected_cm = relative_cm + state->baro_correction_cm;
    if(!isfinite(relative_cm) || !isfinite(corrected_cm))
    {
        return 0;
    }
    state->baro_relative_cm = relative_cm;
    state->corrected_height_cm = corrected_cm;
    return 1;
}
/* 激光辅助气压缓慢校正 */
uint8_t HeightObservation_CorrectBaro(HeightObservation_t *state, float laser_reference_cm, float dt_s)
{
    if (!state->baro_reference_ready ||
        !isfinite(laser_reference_cm) ||
        !isfinite(state->baro_relative_cm) ||
        !isfinite(state->baro_correction_cm) ||
        !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return 0;
    }
    /* 激光观测与未校正气压高度之间的差 */
    float observed_correction_cm = laser_reference_cm - state->baro_relative_cm;  //两者之差 = 气压计当前需要被修正的量
    /* 根据有效观测间隔计算一阶低通系数 */
    float alpha = dt_s / (dt_s + BARO_CORRECTION_TAU_S);
    float correction_cm = state->baro_correction_cm + alpha * (observed_correction_cm - state->baro_correction_cm);
    float corrected_cm = state->baro_relative_cm + correction_cm;
    if(!isfinite(correction_cm) || !isfinite(corrected_cm))
    {
        return 0;
    }
    /* 本轮计算全部成功后，再提交状态 */
    state->baro_correction_cm = correction_cm;
    state->corrected_height_cm = corrected_cm; //最终的高度输出
    return 1;
}
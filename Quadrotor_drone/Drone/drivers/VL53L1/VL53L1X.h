#ifndef VL53L1X_H
#define VL53L1X_H
#include <stdint.h>
/* 初始化：0 成功，非 0 失败 */
int8_t VL53L1X_Init(void);

uint16_t VL53L1X_GetID(void);

/*
 * 返回值：
 *  1：获得新的有效距离，单位 mm
 *  0：暂无新数据
 * -1：接口错误或尚未初始化
 * -2：获得新数据，但测距状态无效
 *
 * 只有返回 1 时才更新 distance_mm。
 * range_status 在成功读取测距状态后更新，0 表示有效。
 */
int8_t VL53L1X_ReadDistance(uint16_t *distance_mm,uint8_t *range_status);

#endif

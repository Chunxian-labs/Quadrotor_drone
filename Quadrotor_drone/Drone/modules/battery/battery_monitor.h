#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

/* 调度器启动前调用一次，初始化ADC和电池状态 */
void BatteryMonitor_Init(void);

/* 仅由电池任务调用：采样、滤波、欠压判断 */
void BatteryMonitor_Update(void);

/* 获取已经发布的电池状态，不执行ADC采样 */
float BatteryMonitor_GetVoltage(void);
uint8_t BatteryMonitor_IsUnderVoltage(void);

/* 飞控提交清除请求，最迟在下一轮电池更新时处理 */
void BatteryMonitor_ClearUnderVoltage(void);

#endif


#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>
void System_timer_Init(void);
/*
 * 阻塞延时：等待期间占用CPU。
 * 初始化阶段和微秒级短等待使用这些接口。
 */
void System_Delay_us(uint32_t us);
void System_Delay_ms(uint32_t ms);
#endif

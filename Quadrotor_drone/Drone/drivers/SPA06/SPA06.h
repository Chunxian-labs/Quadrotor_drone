#ifndef SPA06_H
#define SPA06_H
#include <stdint.h>
int8_t SPA06_Init(void);
uint8_t SPA06_GetID(void);
/*
 *  1：取得新温压数据，单位 Pa、℃
 *  0：暂无完整的新温压数据
 * -1：未初始化、参数错误或运行配置异常
 * -2：补偿结果超出传感器量程或非有限数
 * 返回值不是 1 时，不修改输出。
 */
int8_t SPA06_GetData(float *Pcomp, float *Tcomp);
#endif
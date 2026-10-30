#ifndef __AD_H__
#define __AD_H__
#define AD_CHANNEL_NUM 4
#include <stdint.h>
extern uint16_t AD_Value[AD_CHANNEL_NUM];
void BSP_AD_Init(void);
void BSP_AD_Scan(void);
#endif
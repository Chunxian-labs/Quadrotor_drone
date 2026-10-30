#ifndef FLIGHT_APP_H
#define FLIGHT_APP_H
#include "FreeRTOS.h"
#include "task.h"
#include "telemetry.h"
#include <stdint.h>

/* 遥控包数据 */
typedef struct
{
    uint8_t ID;  /* 0x10普通控制包，0x11请求遥测 */
    int8_t LH;   /* 偏航输入，-100～100 */
    int8_t LV;   /* 油门或垂直速度输入，-100～100 */
    int8_t RH;   /* 横滚输入，-100～100 */
    int8_t RV;   /* 俯仰输入，-100～100 */
    uint8_t KEY; /* 按键码 */

    /* 实际收到有效包的时间 */
    TickType_t received_tick;
} RemoteData_t;

/* 在创建应用任务、启动调度器之前调用 */
uint8_t FlightApp_Init(void);

/* 更新电池状态、接收遥控指令，并处理遥控失联 */
void FlightApp_ProcessInputs(void);

/* 由应用任务每1ms调用一次，内部调度2ms、4ms、8ms功能 */
void FlightApp_Tick1ms(void);

/* 无线任务调用：读取并校验遥控包 */
uint8_t FlightApp_RemoteRead(RemoteData_t *received);

/* 飞控任务调用：应用已经校验的指令 */
void FlightApp_RemoteApply(const RemoteData_t *received);

/* 飞控任务调用：取得飞机当前状态，不执行无线发送 */
void FlightApp_GetTelemetry(TelemetryData_t *data);
#endif

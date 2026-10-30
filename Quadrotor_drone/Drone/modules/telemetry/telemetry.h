#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* 逻辑数据结构，不直接将整个结构体当作无线包发送 */
typedef struct
{
    uint8_t flight_mode;  /* 0自稳、1定高 */
    int16_t throttle;     /* 当前基础油门 */
    float battery_v;     /* 电池电压，V */

    float roll_deg;      /* 横滚角，度 */
    float pitch_deg;     /* 俯仰角，度 */
    float yaw_deg;       /* 偏航角，度 */
    float height_cm;     /* 高度观测，cm */

    uint8_t under_voltage; /* 1欠压 */
    uint8_t run_state;     /* 0停止、1就绪、2运行 */
} TelemetryData_t;

/* 打包并发送；返回现有NRF驱动的发送结果 */
uint8_t Telemetry_Send(const TelemetryData_t *data);

#endif
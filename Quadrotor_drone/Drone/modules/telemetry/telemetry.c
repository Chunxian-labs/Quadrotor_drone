#include "telemetry.h"
#include "NRF24L01.h"
#include <string.h>

/* ==================== 无线包配置 ==================== */

#define TELEMETRY_PACKET_SIZE 32U
#define TELEMETRY_PACKET_ID   0x12U

/* ==================== 打包与发送 ==================== */

uint8_t Telemetry_Send(const TelemetryData_t *data)
{
    memset(NRF24L01_TxPacket, 0, TELEMETRY_PACKET_SIZE);

    NRF24L01_TxPacket[0] = TELEMETRY_PACKET_ID;
    NRF24L01_TxPacket[1] = data->flight_mode;

    memcpy(&NRF24L01_TxPacket[2],
           &data->throttle, sizeof(data->throttle));

    memcpy(&NRF24L01_TxPacket[4],
           &data->battery_v, sizeof(data->battery_v));

    memcpy(&NRF24L01_TxPacket[8],
           &data->roll_deg, sizeof(data->roll_deg));

    memcpy(&NRF24L01_TxPacket[12],
           &data->pitch_deg, sizeof(data->pitch_deg));

    memcpy(&NRF24L01_TxPacket[16],
           &data->yaw_deg, sizeof(data->yaw_deg));

    memcpy(&NRF24L01_TxPacket[20],
           &data->height_cm, sizeof(data->height_cm));

    /* 状态字段 */
    NRF24L01_TxPacket[24] = data->under_voltage;
    NRF24L01_TxPacket[25] = data->run_state;

    return NRF24L01_Send();
}
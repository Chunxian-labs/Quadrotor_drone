#include "Application.h"
#include "joystick.h"
#include "OLED.h"
#include "NRF24L01.h"
#include <string.h>

/* ==================== 周期与协议 ==================== */
#define REMOTE_SEND_PERIOD_MS 20U
#define TELEMETRY_TIMEOUT_MS 3000U /* 本地显示超时，不参与飞机控制 */
#define REMOTE_PACKET_ID 0x11U     /* 每包均请求回传，与显示页面无关 */
#define DRONE_PACKET_ID 0x12U

/* ==================== 遥控输入 ==================== */
static volatile uint8_t SendPending; /* 1ms中断置位，主循环发送 */
static volatile uint32_t SystemMs;   /* 上电毫秒计数 */
static int8_t LH, LV, RH, RV;        /* 四路摇杆，-100~100 */
static uint8_t PendingKey;           /* 待发送的K1~K4命令，K2优先 */
static uint8_t LastKey;              /* 最近一次有效控制按键，供显示 */
static uint8_t LastPressed[BUTTON_NUM]; /* 按键边沿检测 */
static uint8_t SendResult;           /* NRF驱动发送返回值 */
static uint8_t DisplayPage;          /* 0摇杆页，1飞行状态页，K9切换 */

/* ==================== 飞机遥测 ==================== */
typedef struct
{
    uint8_t mode;       /* 0自稳，1定高 */
    int16_t throttle;   /* 基础油门 */
    float battery_v;   /* 电池电压，V */
    float roll_deg;    /* 横滚角，度 */
    float pitch_deg;   /* 俯仰角，度 */
    float yaw_deg;     /* 偏航角，度 */
    float height_cm;   /* 校正后的高度观测，cm */
    uint8_t under_voltage; /* 欠压锁存标志 */
    uint8_t run_state; /* 0停止，1就绪，2运行 */
} DroneTelemetry_t;

static DroneTelemetry_t Drone;
static uint8_t TelemetryReceived; /* 是否收到过有效遥测 */
static uint32_t LastTelemetryMs; /* 最近有效遥测的接收时刻 */

/* ==================== 输入与通信 ==================== */
static void Remote_ReadInput(void)
{
    BSP_AD_Scan();
    LH = DataProcess(AD_Value[0]);
    LV = DataProcess(AD_Value[1]);
    RH = DataProcess(AD_Value[2]);
    RV = DataProcess(AD_Value[3]);

    for (uint8_t i = 0; i < BUTTON_NUM; i++)
    {
        uint8_t pressed = BSP_Button_CheckState((Button_ID)i, BUTTON_PRESSED);
        if (pressed && !LastPressed[i])
        {
            uint8_t key = i + 1U;
            if (key == 2U)
            {
                PendingKey = 2U;
                LastKey = 2U;
                SendPending = 1U; /* 停机请求立即发送 */
            }
            else if (key >= 1U && key <= 4U && PendingKey != 2U)
            {
                PendingKey = key;
                LastKey = key;
            }
            else if (key == 9U)
            {
                DisplayPage = !DisplayPage;
            }
            /* K5~K8、K10~K12不参与本工程，不发送微调命令。 */
        }
        LastPressed[i] = pressed;
    }
}

static void Remote_Send(void)
{
    if (!SendPending)
    {
        return;
    }
    SendPending = 0U;
    memset(NRF24L01_TxPacket, 0, 32U);
    NRF24L01_TxPacket[0] = REMOTE_PACKET_ID;
    NRF24L01_TxPacket[1] = (uint8_t)LH;
    NRF24L01_TxPacket[2] = (uint8_t)LV;
    NRF24L01_TxPacket[3] = (uint8_t)RH;
    NRF24L01_TxPacket[4] = (uint8_t)RV;
    NRF24L01_TxPacket[5] = PendingKey;
    SendResult = NRF24L01_Send();
    if (SendResult == 1U)
    {
        PendingKey = 0U; /* 沿用原有ACK成功后清除命令的方式 */
    }
}

static void Remote_Receive(void)
{
    if (NRF24L01_Receive() != 1U || NRF24L01_RxPacket[0] != DRONE_PACKET_ID)
    {
        return;
    }
    /* 与飞机保持相同字段偏移；memcpy避免未对齐指针强转。 */
    DroneTelemetry_t received = {0};
    received.mode = NRF24L01_RxPacket[1];
    memcpy(&received.throttle, &NRF24L01_RxPacket[2], sizeof(received.throttle));
    memcpy(&received.battery_v, &NRF24L01_RxPacket[4], sizeof(received.battery_v));
    memcpy(&received.roll_deg, &NRF24L01_RxPacket[8], sizeof(received.roll_deg));
    memcpy(&received.pitch_deg, &NRF24L01_RxPacket[12], sizeof(received.pitch_deg));
    memcpy(&received.yaw_deg, &NRF24L01_RxPacket[16], sizeof(received.yaw_deg));
    memcpy(&received.height_cm, &NRF24L01_RxPacket[20], sizeof(received.height_cm));
    received.under_voltage = NRF24L01_RxPacket[24];
    received.run_state = NRF24L01_RxPacket[25];
    if (received.mode > 1U || received.under_voltage > 1U || received.run_state > 2U)
    {
        return;
    }
    Drone = received;
    LastTelemetryMs = SystemMs;
    TelemetryReceived = 1U;
}

/* ==================== OLED显示 ==================== */
static void Remote_Display(void)
{
    uint8_t online = TelemetryReceived &&
        (uint32_t)(SystemMs - LastTelemetryMs) < TELEMETRY_TIMEOUT_MS;
    OLED_Clear();
    /* 两页都显示电压和欠压；超时后不把旧数据当作当前状态。 */
    if (!online)
    {
        OLED_Printf(0, 0, OLED_8X16, "NO TELEMETRY");
    }
    else
    {
        OLED_Printf(0, 0, OLED_8X16, "%4.2fV %s", Drone.battery_v,
                    Drone.under_voltage ? "LOW BAT!" : "BAT OK");
    }

    if (DisplayPage == 0U)
    {
        const char *command = "--";
        switch (LastKey)
        {
            case 1: command = "READY"; break;
            case 2: command = "STOP"; break;
            case 3: command = "STAB"; break;
            case 4: command = "ALT"; break;
            default: break;
        }
        /* CMD表示最近按键，不代表飞机已经执行。 */
        OLED_Printf(0, 16, OLED_8X16, "CMD:%s", command);
        OLED_Printf(0, 32, OLED_8X16, "LH:%+04d RH:%+04d", LH, RH);
        OLED_Printf(0, 48, OLED_8X16, "LV:%+04d RV:%+04d", LV, RV);
    }
    else if (online)
    {
        const char *state = Drone.run_state == 0U ? "STOP" :
                            Drone.run_state == 1U ? "RDY" : "RUN";
        OLED_Printf(0, 16, OLED_8X16, "%s %s T:%d",
                    Drone.mode == 0U ? "STAB" : "ALT", state, Drone.throttle);
        OLED_Printf(0, 32, OLED_8X16, "R:%+04.0f P:%+04.0f", Drone.roll_deg, Drone.pitch_deg);
        OLED_Printf(0, 48, OLED_8X16, "H:%+05.0f Y:%+04.0f", Drone.height_cm, Drone.yaw_deg);
    }
    if (SendResult != 0U && SendResult != 1U)
    {
        /* 状态页第二行或摇杆页命令行显示发送错误，保留欠压提示。 */
        OLED_ClearArea(0, 16, 128, 16);
        OLED_Printf(0, 16, OLED_8X16, "Send Error:%u", (unsigned int)SendResult);
    }
    OLED_Update();
}

int main(void)
{
    OLED_Init();
    Application_Init();
    NRF24L01_Init();
    while (1)
    {
        Remote_ReadInput();
        Remote_Receive(); /* 先取上一轮回传，避免只在某个页面接收 */
        Remote_Send();
        Remote_Receive();
        Remote_Display();
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t send_count;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        SystemMs++;
        BSP_Button_Scan1ms();
        if (++send_count >= REMOTE_SEND_PERIOD_MS)
        {
            send_count = 0U;
            SendPending = 1U;
        }
    }
}


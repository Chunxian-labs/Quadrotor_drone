#include "battery_monitor.h"
#include "bsp_ad.h"
#include "FreeRTOS.h"
#include "task.h"
#define UNDER_VOLTAGE_THRESHOLD_V  2.8f   //电池欠压阈值，单位V
#define BATTERY_FILTER_SAMPLES     10U    //滤波采样点数
#define UNDER_VOLTAGE_CONFIRM_MS   1000U  //欠压判断时间基数

/* 发布给飞控的数据，通过短临界区访问 */
static float BatteryVoltage;
static uint8_t UnderVoltageFlag;

/* 飞控写请求，电池任务取走并处理 */
static uint8_t ClearRequested;

/* 以下状态仅由电池任务修改 */
static float VoltageBuffer[BATTERY_FILTER_SAMPLES];
static uint8_t LowTiming;
static TickType_t LowStartTick;
static uint8_t UnderVoltageLatched;  //低压所存标志，判断是否持续欠压

void BatteryMonitor_Init(void)
{
    // 初始化ADC
    BSP_AD_Init();
    /*
     * 用首次真实测量填满缓存，
     * 避免上电时零缓存造成虚假的低平均电压。
     */
    float voltage = BSP_AD_GetVoltage();

    for (uint8_t i = 0; i < BATTERY_FILTER_SAMPLES; i++)
    {
        VoltageBuffer[i] = voltage;
    }

    BatteryVoltage = voltage;
    UnderVoltageFlag = 0U;
    ClearRequested = 0U;

    LowTiming = 0U;
    LowStartTick = 0U;
    UnderVoltageLatched = 0U;
}
/* 仅由电池任务调用,电压采集与滤波 */
void BatteryMonitor_Update(void)
{
    uint8_t clear_requested;
    /* 取走请求；不能把ADC采样放进临界区 */
    taskENTER_CRITICAL();
    clear_requested = ClearRequested;
    ClearRequested = 0U;
    taskEXIT_CRITICAL();
    if(clear_requested)
    {
        /*
         * 保留已经建立的低压计时，不因按键清除而重置。
         * 如果持续欠压，达到确认条件后仍会再次置位。
         */
        UnderVoltageLatched = 0U;
    }
    float voltage = BSP_AD_GetVoltage();
    float sum = 0.0f;
    //10点滑动平均
    for (uint8_t i = 0; i < BATTERY_FILTER_SAMPLES - 1U; i++)
    {
        VoltageBuffer[i] = VoltageBuffer[i + 1U];
        sum += VoltageBuffer[i];
    }
    VoltageBuffer[BATTERY_FILTER_SAMPLES - 1U] = voltage;
    sum += voltage;
    float filtered_voltage = sum / BATTERY_FILTER_SAMPLES;

    TickType_t now_tick = xTaskGetTickCount();
    if(filtered_voltage < UNDER_VOLTAGE_THRESHOLD_V)
    {
        if(!LowTiming)
        {
            /* 首次检测到低压，开始计时 */
            LowTiming = 1U;
            LowStartTick = now_tick;
        }
        else if((TickType_t)(now_tick - LowStartTick) >= pdMS_TO_TICKS(UNDER_VOLTAGE_CONFIRM_MS))
        {
            /* 持续欠压满1s，确认欠压并锁存 */
            UnderVoltageLatched = 1U;
            //重新开始下一段计时
            LowStartTick = now_tick;
        }
    }
    else
    {
        /* 电压恢复只撤销计时，不自动清除欠压锁存 */
        LowTiming = 0U;
    }
    /* 一次发布电压与欠压状态 */
    taskENTER_CRITICAL();
    BatteryVoltage = filtered_voltage;
    UnderVoltageFlag = UnderVoltageLatched;
    taskEXIT_CRITICAL();
}
float BatteryMonitor_GetVoltage(void)
{
    /*
     * 启动前没有任务并发，不需要进入RTOS临界区。
     * application_start中填充初始遥测时会走这里。
     */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return BatteryVoltage;
    }

    float voltage;

    taskENTER_CRITICAL();
    voltage = BatteryVoltage;
    taskEXIT_CRITICAL();

    return voltage;
}
uint8_t BatteryMonitor_IsUnderVoltage(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return UnderVoltageFlag;
    }

    uint8_t flag;

    taskENTER_CRITICAL();
    flag = UnderVoltageFlag;
    taskEXIT_CRITICAL();

    return flag;
}

void BatteryMonitor_ClearUnderVoltage(void)
{
    /* 只提交请求，不直接修改电池任务内部状态 */
    taskENTER_CRITICAL();
    ClearRequested = 1U;
    taskEXIT_CRITICAL();
}
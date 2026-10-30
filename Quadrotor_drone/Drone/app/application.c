#include "application.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "flight_app.h"
#include "battery_monitor.h"
/* 飞控任务栈大小4096字节，优先级为4 */
#define FLIGHT_TASK_STACK_SIZE  1024U
#define FLIGHT_TASK_PRIORITY    4U
/* 无线收发任务栈大小1024字节，优先级为3U */
#define RADIO_TASK_STACK_SIZE  256U
#define RADIO_TASK_PRIORITY    3U
/* 电池监控任务栈大小512字节，优先级为1U */
#define BATTERY_TASK_STACK_SIZE  128U
#define BATTERY_TASK_PRIORITY    1U

/* 无线数据存储队列，长度为2，超时时间3s*/
#define REMOTE_QUEUE_LENGTH    2U
#define REMOTE_MAX_AGE_MS      3000U
/* 唯一的遥控队列，由FreeRTOS管理内部存储 */
static QueueHandle_t remote_queue;

/* 唯一一份共享遥测快照，复制期间使用短临界区 */
static TelemetryData_t telemetry_snapshot;
/* 飞控任务：处理指令、执行控制、发布状态 */
static void flight_task(void *pvParameters)
{
    (void)pvParameters;
    /* 记录周期调度的起始节拍 */
    TickType_t last_tick = xTaskGetTickCount();
    RemoteData_t received_data;  //遥控器端发送数据，遥控器 --> 飞行器
    TelemetryData_t snapshot;    //飞机端保存的遥测数据照，飞行器 --> 遥控器
    while(1)
    {
        /*
         * 先等待下一个1ms周期。
         * 首次控制计算也在调度器启动约1ms后执行。
         */
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(1));
        //不等待收包，每轮最多取两包数据
        for(BaseType_t i = 0; i < REMOTE_QUEUE_LENGTH; ++i)
        {
            if (xQueueReceive(remote_queue, &received_data, 0) != pdPASS)
            {
                break;
            }
            TickType_t data_age = (TickType_t)(xTaskGetTickCount() - received_data.received_tick);
            if (data_age > pdMS_TO_TICKS(REMOTE_MAX_AGE_MS) && received_data.KEY != 2U) //不执行排队过久的数据，但如果存在停止指令就需要执行
            {
                continue;
            }
            // 处理数据并执行控制
            FlightApp_RemoteApply(&received_data);
        }
        /*
         * 遥控失联处理。
         */
        FlightApp_ProcessInputs();
        FlightApp_Tick1ms();

        /*
         * 在临界区外整理状态。
         * 临界区内只复制结构体，不能执行采样或无线发送。
         */
        FlightApp_GetTelemetry(&snapshot);

        taskENTER_CRITICAL();
        telemetry_snapshot = snapshot;
        taskEXIT_CRITICAL();
    }
}
/* 无线任务：收包、提交指令，按请求回复遥测 */
static void radio_task(void *pvParameters)
{
    (void)pvParameters;
    RemoteData_t received_data;
    TelemetryData_t snapshot;
    while(1)
    {
        if(FlightApp_RemoteRead(&received_data))
        {
            /*
             * 请求遥测时，先保存最近一次飞控状态。
             * 不等待飞控处理当前包。
             */
            if(received_data.ID == 0x11U)//说明远程需要请求遥测数据
            {
                taskENTER_CRITICAL();
                snapshot = telemetry_snapshot;
                taskEXIT_CRITICAL();
            }
            /*
             * 队列满则阻塞无线任务，等飞控取走数据。
             * 不覆盖旧包，避免覆盖其中的按键指令。
             */
            if(xQueueSend(remote_queue, &received_data, portMAX_DELAY) == pdPASS)
            {
                if(received_data.ID == 0x11U)
                {
                    /* 发送期间不进入临界区，飞控可以抢占 */
                    (void)Telemetry_Send(&snapshot);
                }
            }
        }
        /* 普通轮询等待，避免无线任务持续占用CPU */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
static void battery_task(void *pvParameters)
{
    (void)pvParameters;
    while(1)
    {
        BatteryMonitor_Update();
        /*
         * 采样完成后阻塞10ms。
         * 欠压使用实际节拍计时，不依赖任务执行次数。
         * 不追赶错过的采样周期。
         */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/* 创建应用任务,并保存数据快照 */
int application_start(void)
{
    remote_queue = xQueueCreate(REMOTE_QUEUE_LENGTH, sizeof(RemoteData_t));
    if(remote_queue == NULL)
    {
        return 0;
    }
    //先填充初始快照
    FlightApp_GetTelemetry(&telemetry_snapshot);
    if(xTaskCreate(flight_task, 
            "flight_task", 
            FLIGHT_TASK_STACK_SIZE, 
            NULL, 
            FLIGHT_TASK_PRIORITY, 
            NULL) != pdPASS)
    {
        return 0;
    }
    if(xTaskCreate(radio_task, 
            "radio_task", 
            RADIO_TASK_STACK_SIZE, 
            NULL, 
            RADIO_TASK_PRIORITY, 
            NULL) != pdPASS)
    {
        return 0;
    }
    if(xTaskCreate(battery_task, 
            "battery_task", 
            BATTERY_TASK_STACK_SIZE, 
            NULL, 
            BATTERY_TASK_PRIORITY, 
            NULL) != pdPASS)
    {
        return 0;
    }
    return 1;
}

#include "FreeRTOS.h"
#include "task.h"
#include "application.h"
#include "bsp_timer.h"
#include "flight_app.h"
#include "motor.h"

/* 故障类型，可在调试器中查看 g_app_fault。 */
typedef enum
{
    APP_FAULT_NONE = 0,
    APP_FAULT_STARTUP = 1,
    APP_FAULT_ASSERT = 2,
    APP_FAULT_MALLOC = 3,
    APP_FAULT_STACK = 4
} AppFaultReason;

typedef struct
{
    AppFaultReason reason;
    const char *file;       /* 断言所在文件；非断言故障为NULL */
    int line;              /* 断言所在行号 */
    TaskHandle_t task;     /* 栈溢出任务句柄 */
    const char *task_name; /* 仅保存名称指针，故障路径不解引用 */
} AppFaultInfo;

/* 保存在RAM中供调试器查看，复位后清零，不写入Flash。 */
volatile AppFaultInfo g_app_fault;

static void app_fault_stop(AppFaultReason reason, const char *file,
                           int line, TaskHandle_t task,
                           const char *task_name) __attribute__((noreturn));

/* 故障路径不打印、不分配内存、不调用任务调度接口。 */
static void app_fault_stop(AppFaultReason reason, const char *file,
                           int line, TaskHandle_t task,
                           const char *task_name)
{
    /* 禁止中断及任务切换，防止其他执行路径恢复电机输出。 */
    __disable_irq();
    Motor_Stop();

    g_app_fault.reason = reason;
    g_app_fault.file = file;
    g_app_fault.line = line;
    g_app_fault.task = task;
    g_app_fault.task_name = task_name;

    __DSB();
    while(1)
    {
        __NOP();
    }
}

/* 保留原有启动失败入口，统一走同一停机流程。 */
static void startup_failed(void)
{
    app_fault_stop(APP_FAULT_STARTUP, NULL, 0, NULL, NULL);
}

void vAssertCalled(const char *file, int line)
{
    app_fault_stop(APP_FAULT_ASSERT, file, line, NULL, NULL);
}

void vApplicationMallocFailedHook(void)
{
    app_fault_stop(APP_FAULT_MALLOC, NULL, 0, NULL, NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    /* 栈可能已损坏，不复制任务名，不再调用FreeRTOS查询接口。 */
    app_fault_stop(APP_FAULT_STACK, NULL, 0, task, task_name);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /* 保留TIM3微秒计时，为初始化和校准提供阻塞延时 */
    System_timer_Init();
    /* 校准失败：电机保持停止，不启动飞行调度；重新上电再校准 */
    if(!FlightApp_Init())
    {
        startup_failed();
    }
    /* 创建应用任务，检查是否分配成功 */
    if(!application_start())
    {
        startup_failed();
    }
    /*
     * 启动FreeRTOS调度器。
     * 成功后开始执行任务，正常情况下不会返回。
     */
    vTaskStartScheduler();

    /* 如果返回，说明调度器启动失败 */
    startup_failed();
    while(1)
    {
        ;
    }
}

/*
 * FreeRTOS V202212.01
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Library includes. */
#include "stm32f10x.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ			( ( unsigned long ) SystemCoreClock )
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 5 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 12 * 1024 ) )
#define configMAX_TASK_NAME_LEN		( 16 )
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1
#define configUSE_MUTEXES			1


/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_xTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

/* STM32F103 implements four NVIC priority bits.
 * Requires NVIC_PriorityGroup_4 when the kernel is integrated.
 * ISRs using FreeRTOS FromISR APIs must have logical priorities 5..15.
 * Higher urgency ISRs (0..4) must not call FreeRTOS APIs.
 */
#define configPRIO_BITS                                  4
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY          15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_KERNEL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* FreeRTOS Cortex-M3 interrupt handler mapping. */
#define xPortSysTickHandler SysTick_Handler
#define xPortPendSVHandler    PendSV_Handler
#define vPortSVCHandler       SVC_Handler

#define INCLUDE_xTaskGetSchedulerState    1

/* Current RAM budget for the 20 KB F103C8: 12 KB RTOS heap.
 * Dynamic task stacks and RTOS objects are allocated from this heap.
 * configMINIMAL_STACK_SIZE is in 32-bit words (128 words = 512 bytes).
 * Revisit the heap and individual task stacks after measuring actual use.
 */
#define configSUPPORT_STATIC_ALLOCATION     0
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configAPPLICATION_ALLOCATED_HEAP    0

#define configUSE_TIME_SLICING              1
#define configUSE_TICKLESS_IDLE             0
#define configUSE_TASK_NOTIFICATIONS        1
#define configUSE_TIMERS                    0
#define configUSE_CO_ROUTINES               0

/* Fault handlers are implemented in app/main.c; they stop PWM and halt. */
#define configCHECK_FOR_STACK_OVERFLOW      2
#define configUSE_MALLOC_FAILED_HOOK         1
#define INCLUDE_uxTaskGetStackHighWaterMark  1

/* Capture the failed assertion location without printf or allocation. */
void vAssertCalled(const char *file, int line);
#define configASSERT(condition)                         \
    do                                                 \
    {                                                  \
        if (!(condition))                              \
        {                                              \
            vAssertCalled(__FILE__, __LINE__);          \
        }                                              \
    } while (0)

#endif /* FREERTOS_CONFIG_H */


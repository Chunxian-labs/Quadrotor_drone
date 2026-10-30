.syntax unified
.cpu cortex-m3
.thumb

.global g_pfnVectors
.global Default_Handler
.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
  ldr sp, =_estack
  bl SystemInit
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
1: cmp r0, r1
   bcs 2f
   ldr r3, [r2], #4
   str r3, [r0], #4
   b 1b
2: ldr r0, =_sbss
   ldr r1, =_ebss
   movs r2, #0
3: cmp r0, r1
   bcs 4f
   str r2, [r0], #4
   b 3b
4: bl __libc_init_array
   bl main
5: b 5b
.size Reset_Handler, .-Reset_Handler

.section .text.Default_Handler,"ax",%progbits
Default_Handler: b .

.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
g_pfnVectors:
  .word _estack, Reset_Handler, NMI_Handler, HardFault_Handler
  .word MemManage_Handler, BusFault_Handler, UsageFault_Handler, 0, 0, 0, 0
  .word SVC_Handler, DebugMon_Handler, 0, PendSV_Handler, SysTick_Handler
  .word WWDG_IRQHandler, PVD_IRQHandler, TAMPER_IRQHandler, RTC_IRQHandler
  .word FLASH_IRQHandler, RCC_IRQHandler, EXTI0_IRQHandler, EXTI1_IRQHandler
  .word EXTI2_IRQHandler, EXTI3_IRQHandler, EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler, DMA1_Channel2_IRQHandler, DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler, DMA1_Channel5_IRQHandler, DMA1_Channel6_IRQHandler, DMA1_Channel7_IRQHandler
  .word ADC1_2_IRQHandler, USB_HP_CAN1_TX_IRQHandler, USB_LP_CAN1_RX0_IRQHandler
  .word CAN1_RX1_IRQHandler, CAN1_SCE_IRQHandler, EXTI9_5_IRQHandler
  .word TIM1_BRK_IRQHandler, TIM1_UP_IRQHandler, TIM1_TRG_COM_IRQHandler, TIM1_CC_IRQHandler
  .word TIM2_IRQHandler, TIM3_IRQHandler, TIM4_IRQHandler, I2C1_EV_IRQHandler, I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler, I2C2_ER_IRQHandler, SPI1_IRQHandler, SPI2_IRQHandler
  .word USART1_IRQHandler, USART2_IRQHandler, USART3_IRQHandler, EXTI15_10_IRQHandler
  .word RTCAlarm_IRQHandler, USBWakeUp_IRQHandler
.size g_pfnVectors, .-g_pfnVectors

.macro weak_handler name
  .weak \name
  .thumb_set \name, Default_Handler
.endm
weak_handler NMI_Handler
weak_handler HardFault_Handler
weak_handler MemManage_Handler
weak_handler BusFault_Handler
weak_handler UsageFault_Handler
weak_handler SVC_Handler
weak_handler DebugMon_Handler
weak_handler PendSV_Handler
weak_handler SysTick_Handler
weak_handler WWDG_IRQHandler
weak_handler PVD_IRQHandler
weak_handler TAMPER_IRQHandler
weak_handler RTC_IRQHandler
weak_handler FLASH_IRQHandler
weak_handler RCC_IRQHandler
weak_handler EXTI0_IRQHandler
weak_handler EXTI1_IRQHandler
weak_handler EXTI2_IRQHandler
weak_handler EXTI3_IRQHandler
weak_handler EXTI4_IRQHandler
weak_handler DMA1_Channel1_IRQHandler
weak_handler DMA1_Channel2_IRQHandler
weak_handler DMA1_Channel3_IRQHandler
weak_handler DMA1_Channel4_IRQHandler
weak_handler DMA1_Channel5_IRQHandler
weak_handler DMA1_Channel6_IRQHandler
weak_handler DMA1_Channel7_IRQHandler
weak_handler ADC1_2_IRQHandler
weak_handler USB_HP_CAN1_TX_IRQHandler
weak_handler USB_LP_CAN1_RX0_IRQHandler
weak_handler CAN1_RX1_IRQHandler
weak_handler CAN1_SCE_IRQHandler
weak_handler EXTI9_5_IRQHandler
weak_handler TIM1_BRK_IRQHandler
weak_handler TIM1_UP_IRQHandler
weak_handler TIM1_TRG_COM_IRQHandler
weak_handler TIM1_CC_IRQHandler
weak_handler TIM2_IRQHandler
weak_handler TIM3_IRQHandler
weak_handler TIM4_IRQHandler
weak_handler I2C1_EV_IRQHandler
weak_handler I2C1_ER_IRQHandler
weak_handler I2C2_EV_IRQHandler
weak_handler I2C2_ER_IRQHandler
weak_handler SPI1_IRQHandler
weak_handler SPI2_IRQHandler
weak_handler USART1_IRQHandler
weak_handler USART2_IRQHandler
weak_handler USART3_IRQHandler
weak_handler EXTI15_10_IRQHandler
weak_handler RTCAlarm_IRQHandler
weak_handler USBWakeUp_IRQHandler

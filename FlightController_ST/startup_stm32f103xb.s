  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
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
  ldr   sp, =_estack

  ldr   r0, =_sdata
  ldr   r1, =_edata
  movs  r2, #0
  b     LoopCopyDataInit

CopyDataInit:
  ldr   r3, =_sidata
  ldr   r3, [r3, r2]
  str   r3, [r0, r2]
  adds  r2, r2, #4

LoopCopyDataInit:
  adds  r3, r0, r2
  cmp   r3, r1
  bcc   CopyDataInit

  ldr   r2, =_sbss
  ldr   r4, =_ebss
  movs  r3, #0
  b     LoopFillZerobss

FillZerobss:
  str   r3, [r2]
  adds  r2, r2, #4

LoopFillZerobss:
  cmp   r2, r4
  bcc   FillZerobss

  bl    SystemInit
  bl    main
  bx    lr
.size Reset_Handler, .-Reset_Handler

  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word Default_Handler /* WWDG */
  .word Default_Handler /* PVD */
  .word Default_Handler /* TAMPER */
  .word Default_Handler /* RTC */
  .word Default_Handler /* FLASH */
  .word Default_Handler /* RCC */
  .word Default_Handler /* EXTI0 */
  .word Default_Handler /* EXTI1 */
  .word Default_Handler /* EXTI2 */
  .word Default_Handler /* EXTI3 */
  .word Default_Handler /* EXTI4 */
  .word Default_Handler /* DMA1_Channel1 */
  .word Default_Handler /* DMA1_Channel2 */
  .word Default_Handler /* DMA1_Channel3 */
  .word Default_Handler /* DMA1_Channel4 */
  .word Default_Handler /* DMA1_Channel5 */
  .word Default_Handler /* DMA1_Channel6 */
  .word Default_Handler /* DMA1_Channel7 */
  .word Default_Handler /* ADC1_2 */
  .word Default_Handler /* USB_HP_CAN1_TX */
  .word Default_Handler /* USB_LP_CAN1_RX0 */
  .word Default_Handler /* CAN1_RX1 */
  .word Default_Handler /* CAN1_SCE */
  .word Default_Handler /* EXTI9_5 */
  .word Default_Handler /* TIM1_BRK */
  .word Default_Handler /* TIM1_UP */
  .word Default_Handler /* TIM1_TRG_COM */
  .word Default_Handler /* TIM1_CC */
  .word Default_Handler /* TIM2 */
  .word Default_Handler /* TIM3 */
  .word Default_Handler /* TIM4 */
  .word Default_Handler /* I2C1_EV */
  .word Default_Handler /* I2C1_ER */
  .word Default_Handler /* I2C2_EV */
  .word Default_Handler /* I2C2_ER */
  .word Default_Handler /* SPI1 */
  .word Default_Handler /* SPI2 */
  .word Default_Handler /* USART1 */
  .word Default_Handler /* USART2 */
  .word Default_Handler /* USART3 */
  .word Default_Handler /* EXTI15_10 */
  .word Default_Handler /* RTCAlarm */
  .word Default_Handler /* USBWakeUp */

  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler
  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler
  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler
  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler
  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler
  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler
  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler
  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler
  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

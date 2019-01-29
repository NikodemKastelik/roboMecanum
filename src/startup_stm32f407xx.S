.syntax unified
.cpu    cortex-m4
.thumb

.global g_pfnVectors
.global Default_Handler

/**
 * @brief Symbol for determining address of .data section initial values block.
 */
.word _data_values_start

/**
 * @brief Symbol for determining start address of .data section.
 */
.word _data_start

/**
 * @brief Symbol for end address of .data section.
 */
.word _data_end

/**
 * @brief Symbol for determining start address of .bss section.
 */
.word _bss_start

/**
 * @brief Symbol for end address of .bss section.
 */
.word _bss_end

/**
 * @brief Application entry code.
 *
 * CPU starts execution from here just after reset. This routine prepares static
and global variables. Then jump to main() is performed and user application
begins execution.
 */
.section .text.Reset_Handler
.weak    Reset_Handler,
.type    Reset_Handler, %function
Reset_Handler:
    /* Set SP to the end of RAM. */
    ldr   r0, =_RAM_END
    mov   sp, r0

    /* Copy data segment initial values from Flash to RAM. */
    ldr   r0, =_data_start
    ldr   r1, =_data_end
    ldr   r2, =_data_values_start
    movs  r3, #0

    /* R1 contains bytes to copy. */
    subs  r1, r1, r0

    b     copy_data_check

copy_data_loop:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, #4

copy_data_check:
    cmp   r3, r1
    bcc   copy_data_loop

    /* Zero fill BSS section. */
    ldr   r0, =_bss_start
    ldr   r1, =_bss_end
    movs  r2, #0
    b     zero_bss_check

zero_bss_loop:
    str   r2, [r0]
    adds  r0, #4

zero_bss_check:
    cmp   r0, r1
    bcc   zero_bss_loop

/* Call user application main() */
    b     main

.size Reset_Handler, .-Reset_Handler

/**
 * brief Function for catching default interrupts.
 *
 * Any interrupt handler not overriden explicitely will be trapped inside this function for debugger to examine.
 */
.section .text.Default_Handler,"ax",%progbits
Default_Handler:
    b   Default_Handler
.size     Default_Handler, .-Default_Handler

/**
 * @brief Vector table.
 */
.section .isr_vector,"a",%progbits
.type    g_pfnVectors, %object
.size    g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _RAM_END
    .word Reset_Handler
    .word NonMaskableInt_Handler
    .word 0
    .word MemoryManagement_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVCall_Handler
    .word DebugMonitor_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    .word WWDG_Handler
    .word PVD_Handler
    .word TAMP_STAMP_Handler
    .word RTC_WKUP_Handler
    .word FLASH_Handler
    .word RCC_Handler
    .word EXTI0_Handler
    .word EXTI1_Handler
    .word EXTI2_Handler
    .word EXTI3_Handler
    .word EXTI4_Handler
    .word DMA1_Stream0_Handler
    .word DMA1_Stream1_Handler
    .word DMA1_Stream2_Handler
    .word DMA1_Stream3_Handler
    .word DMA1_Stream4_Handler
    .word DMA1_Stream5_Handler
    .word DMA1_Stream6_Handler
    .word ADC_Handler
    .word CAN1_TX_Handler
    .word CAN1_RX0_Handler
    .word CAN1_RX1_Handler
    .word CAN1_SCE_Handler
    .word EXTI9_5_Handler
    .word TIM1_BRK_TIM9_Handler
    .word TIM1_UP_TIM10_Handler
    .word TIM1_TRG_COM_TIM11_Handler
    .word TIM1_CC_Handler
    .word TIM2_Handler
    .word TIM3_Handler
    .word TIM4_Handler
    .word I2C1_EV_Handler
    .word I2C1_ER_Handler
    .word I2C2_EV_Handler
    .word I2C2_ER_Handler
    .word SPI1_Handler
    .word SPI2_Handler
    .word USART1_Handler
    .word USART2_Handler
    .word USART3_Handler
    .word EXTI15_10_Handler
    .word RTC_Alarm_Handler
    .word OTG_FS_WKUP_Handler
    .word TIM8_BRK_TIM12_Handler
    .word TIM8_UP_TIM13_Handler
    .word TIM8_TRG_COM_TIM14_Handler
    .word TIM8_CC_Handler
    .word DMA1_Stream7_Handler
    .word FSMC_Handler
    .word SDIO_Handler
    .word TIM5_Handler
    .word SPI3_Handler
    .word UART4_Handler
    .word UART5_Handler
    .word TIM6_DAC_Handler
    .word TIM7_Handler
    .word DMA2_Stream0_Handler
    .word DMA2_Stream1_Handler
    .word DMA2_Stream2_Handler
    .word DMA2_Stream3_Handler
    .word DMA2_Stream4_Handler
    .word ETH_Handler
    .word ETH_WKUP_Handler
    .word CAN2_TX_Handler
    .word CAN2_RX0_Handler
    .word CAN2_RX1_Handler
    .word CAN2_SCE_Handler
    .word OTG_FS_Handler
    .word DMA2_Stream5_Handler
    .word DMA2_Stream6_Handler
    .word DMA2_Stream7_Handler
    .word USART6_Handler
    .word I2C3_EV_Handler
    .word I2C3_ER_Handler
    .word OTG_HS_EP1_OUT_Handler
    .word OTG_HS_EP1_IN_Handler
    .word OTG_HS_WKUP_Handler
    .word OTG_HS_Handler
    .word DCMI_Handler
    .word 0
    .word HASH_RNG_Handler
    .word FPU_Handler

/**
 * @brief Weak aliases for interrupt handlers.
 *
 * Interrupt handlers points to Default_Hander by default. As they are weak
 * aliases, they can be overriden and therefore be used in user code.
 */

.weak      NonMaskableInt_Handler
.thumb_set NonMaskableInt_Handler,Default_Handler

.weak      MemoryManagement_Handler
.thumb_set MemoryManagement_Handler,Default_Handler

.weak      BusFault_Handler
.thumb_set BusFault_Handler,Default_Handler

.weak      UsageFault_Handler
.thumb_set UsageFault_Handler,Default_Handler

.weak      SVCall_Handler
.thumb_set SVCall_Handler,Default_Handler

.weak      DebugMonitor_Handler
.thumb_set DebugMonitor_Handler,Default_Handler

.weak      PendSV_Handler
.thumb_set PendSV_Handler,Default_Handler

.weak      SysTick_Handler
.thumb_set SysTick_Handler,Default_Handler

.weak      WWDG_Handler
.thumb_set WWDG_Handler,Default_Handler

.weak      PVD_Handler
.thumb_set PVD_Handler,Default_Handler

.weak      TAMP_STAMP_Handler
.thumb_set TAMP_STAMP_Handler,Default_Handler

.weak      RTC_WKUP_Handler
.thumb_set RTC_WKUP_Handler,Default_Handler

.weak      FLASH_Handler
.thumb_set FLASH_Handler,Default_Handler

.weak      RCC_Handler
.thumb_set RCC_Handler,Default_Handler

.weak      EXTI0_Handler
.thumb_set EXTI0_Handler,Default_Handler

.weak      EXTI1_Handler
.thumb_set EXTI1_Handler,Default_Handler

.weak      EXTI2_Handler
.thumb_set EXTI2_Handler,Default_Handler

.weak      EXTI3_Handler
.thumb_set EXTI3_Handler,Default_Handler

.weak      EXTI4_Handler
.thumb_set EXTI4_Handler,Default_Handler

.weak      DMA1_Stream0_Handler
.thumb_set DMA1_Stream0_Handler,Default_Handler

.weak      DMA1_Stream1_Handler
.thumb_set DMA1_Stream1_Handler,Default_Handler

.weak      DMA1_Stream2_Handler
.thumb_set DMA1_Stream2_Handler,Default_Handler

.weak      DMA1_Stream3_Handler
.thumb_set DMA1_Stream3_Handler,Default_Handler

.weak      DMA1_Stream4_Handler
.thumb_set DMA1_Stream4_Handler,Default_Handler

.weak      DMA1_Stream5_Handler
.thumb_set DMA1_Stream5_Handler,Default_Handler

.weak      DMA1_Stream6_Handler
.thumb_set DMA1_Stream6_Handler,Default_Handler

.weak      ADC_Handler
.thumb_set ADC_Handler,Default_Handler

.weak      CAN1_TX_Handler
.thumb_set CAN1_TX_Handler,Default_Handler

.weak      CAN1_RX0_Handler
.thumb_set CAN1_RX0_Handler,Default_Handler

.weak      CAN1_RX1_Handler
.thumb_set CAN1_RX1_Handler,Default_Handler

.weak      CAN1_SCE_Handler
.thumb_set CAN1_SCE_Handler,Default_Handler

.weak      EXTI9_5_Handler
.thumb_set EXTI9_5_Handler,Default_Handler

.weak      TIM1_BRK_TIM9_Handler
.thumb_set TIM1_BRK_TIM9_Handler,Default_Handler

.weak      TIM1_UP_TIM10_Handler
.thumb_set TIM1_UP_TIM10_Handler,Default_Handler

.weak      TIM1_TRG_COM_TIM11_Handler
.thumb_set TIM1_TRG_COM_TIM11_Handler,Default_Handler

.weak      TIM1_CC_Handler
.thumb_set TIM1_CC_Handler,Default_Handler

.weak      TIM2_Handler
.thumb_set TIM2_Handler,Default_Handler

.weak      TIM3_Handler
.thumb_set TIM3_Handler,Default_Handler

.weak      TIM4_Handler
.thumb_set TIM4_Handler,Default_Handler

.weak      I2C1_EV_Handler
.thumb_set I2C1_EV_Handler,Default_Handler

.weak      I2C1_ER_Handler
.thumb_set I2C1_ER_Handler,Default_Handler

.weak      I2C2_EV_Handler
.thumb_set I2C2_EV_Handler,Default_Handler

.weak      I2C2_ER_Handler
.thumb_set I2C2_ER_Handler,Default_Handler

.weak      SPI1_Handler
.thumb_set SPI1_Handler,Default_Handler

.weak      SPI2_Handler
.thumb_set SPI2_Handler,Default_Handler

.weak      USART1_Handler
.thumb_set USART1_Handler,Default_Handler

.weak      USART2_Handler
.thumb_set USART2_Handler,Default_Handler

.weak      USART3_Handler
.thumb_set USART3_Handler,Default_Handler

.weak      EXTI15_10_Handler
.thumb_set EXTI15_10_Handler,Default_Handler

.weak      RTC_Alarm_Handler
.thumb_set RTC_Alarm_Handler,Default_Handler

.weak      OTG_FS_WKUP_Handler
.thumb_set OTG_FS_WKUP_Handler,Default_Handler

.weak      TIM8_BRK_TIM12_Handler
.thumb_set TIM8_BRK_TIM12_Handler,Default_Handler

.weak      TIM8_UP_TIM13_Handler
.thumb_set TIM8_UP_TIM13_Handler,Default_Handler

.weak      TIM8_TRG_COM_TIM14_Handler
.thumb_set TIM8_TRG_COM_TIM14_Handler,Default_Handler

.weak      TIM8_CC_Handler
.thumb_set TIM8_CC_Handler,Default_Handler

.weak      DMA1_Stream7_Handler
.thumb_set DMA1_Stream7_Handler,Default_Handler

.weak      FSMC_Handler
.thumb_set FSMC_Handler,Default_Handler

.weak      SDIO_Handler
.thumb_set SDIO_Handler,Default_Handler

.weak      TIM5_Handler
.thumb_set TIM5_Handler,Default_Handler

.weak      SPI3_Handler
.thumb_set SPI3_Handler,Default_Handler

.weak      UART4_Handler
.thumb_set UART4_Handler,Default_Handler

.weak      UART5_Handler
.thumb_set UART5_Handler,Default_Handler

.weak      TIM6_DAC_Handler
.thumb_set TIM6_DAC_Handler,Default_Handler

.weak      TIM7_Handler
.thumb_set TIM7_Handler,Default_Handler

.weak      DMA2_Stream0_Handler
.thumb_set DMA2_Stream0_Handler,Default_Handler

.weak      DMA2_Stream1_Handler
.thumb_set DMA2_Stream1_Handler,Default_Handler

.weak      DMA2_Stream2_Handler
.thumb_set DMA2_Stream2_Handler,Default_Handler

.weak      DMA2_Stream3_Handler
.thumb_set DMA2_Stream3_Handler,Default_Handler

.weak      DMA2_Stream4_Handler
.thumb_set DMA2_Stream4_Handler,Default_Handler

.weak      ETH_Handler
.thumb_set ETH_Handler,Default_Handler

.weak      ETH_WKUP_Handler
.thumb_set ETH_WKUP_Handler,Default_Handler

.weak      CAN2_TX_Handler
.thumb_set CAN2_TX_Handler,Default_Handler

.weak      CAN2_RX0_Handler
.thumb_set CAN2_RX0_Handler,Default_Handler

.weak      CAN2_RX1_Handler
.thumb_set CAN2_RX1_Handler,Default_Handler

.weak      CAN2_SCE_Handler
.thumb_set CAN2_SCE_Handler,Default_Handler

.weak      OTG_FS_Handler
.thumb_set OTG_FS_Handler,Default_Handler

.weak      DMA2_Stream5_Handler
.thumb_set DMA2_Stream5_Handler,Default_Handler

.weak      DMA2_Stream6_Handler
.thumb_set DMA2_Stream6_Handler,Default_Handler

.weak      DMA2_Stream7_Handler
.thumb_set DMA2_Stream7_Handler,Default_Handler

.weak      USART6_Handler
.thumb_set USART6_Handler,Default_Handler

.weak      I2C3_EV_Handler
.thumb_set I2C3_EV_Handler,Default_Handler

.weak      I2C3_ER_Handler
.thumb_set I2C3_ER_Handler,Default_Handler

.weak      OTG_HS_EP1_OUT_Handler
.thumb_set OTG_HS_EP1_OUT_Handler,Default_Handler

.weak      OTG_HS_EP1_IN_Handler
.thumb_set OTG_HS_EP1_IN_Handler,Default_Handler

.weak      OTG_HS_WKUP_Handler
.thumb_set OTG_HS_WKUP_Handler,Default_Handler

.weak      OTG_HS_Handler
.thumb_set OTG_HS_Handler,Default_Handler

.weak      DCMI_Handler
.thumb_set DCMI_Handler,Default_Handler

.weak      HASH_RNG_Handler
.thumb_set HASH_RNG_Handler,Default_Handler

.weak      FPU_Handler
.thumb_set FPU_Handler,Default_Handler


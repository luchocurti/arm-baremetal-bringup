/* startup.c */
/* Startup code, Vector table and Handlers */

#include <stdint.h>

extern int main(void);

/**
 * @file startup.c
 * @brief Cortex-M startup code: vector table, default/weak handlers and reset
 *        runtime initialization (.data/.bss setup) before jumping to main().
 *
 * This file provides a minimal, portable startup implementation which is
 * suitable for small STM32 (Cortex-M) projects. Handlers are declared as weak
 * aliases to `Default_Handler` so the application may override any of them by
 * providing a function with the same name in user code (no link flags needed).
 *
 * The linker must provide the following symbols (typically defined in link.ld):
 *  - _estack  : initial stack pointer (placed at vector table entry 0)
 *  - _sidata  : start address of the initialized data values in flash
 *  - _sdata   : start address of the .data section in RAM
 *  - _edata   : end address of the .data section in RAM
 *  - _sbss    : start address of the .bss section in RAM
 *  - _ebss    : end address of the .bss section in RAM
 */

/* Function prototypes for exception handlers */
void Reset_Handler(void);
void Default_Handler(void);

/* Weak aliases for exception handlers
 *
 * If the application provides a function with the same name (for example
 * `void HardFault_Handler(void)`), the strong symbol in the application will
 * override these weak aliases and the application handler will be used.
 */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* Device IRQ handlers (weak aliases to Default_Handler) */
/* Only override those implemented. Provide weak aliases for the rest so the
 * vector table can safely reference them. */
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_PVM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_TX_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler(void)
    __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* USART2 IRQ prototype */
/* not weak — Implemented it in main.c */
void USART2_IRQHandler(void);

/* Other device IRQ prototypes */

/* Symbols defined in link.ld */
extern uint32_t _sidata; /* Start of .data in FLASH (initial values) */
extern uint32_t _sdata;  /* Start of .data in RAM (destination) */
extern uint32_t _edata;  /* End of .data in RAM */
extern uint32_t _sbss;   /* Start of .bss in RAM */
extern uint32_t _ebss;   /* End of .bss in RAM */
extern uint32_t _estack; /* Top of stack (initial SP) */

/* NUM_DEVICE_IRQS: Choose at least the highest device IRQ used.
   On STM32L476 there are ~82 device IRQs; using 82 is safe for USART2 (IRQ#38).
 */
#define NUM_DEVICE_IRQS 82
#define TOTAL_VECTORS (16 + NUM_DEVICE_IRQS)

/*
 * Vector table placed into the .isr_vector section (linker script must
 * place this section at the device's vector table base address, typically
 * 0x08000000 in flash). Index 0 contains the initial stack pointer value
 * (SP) and index 1 contains the address of the Reset_Handler.
 *
 * The array is sparse-initialized: only entries we care about are listed.
 */
__attribute__((section(".isr_vector"))) void (*const g_pfnVectors[])(void) = {
    [0] = (void (*)(void))(&_estack),
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 + 38] = USART2_IRQHandler, /* 38 is USART2 IRQ number */
};

/* ===================================================================== */
/* Reset Handler: perform C runtime initialization and call main()         */
/* ===================================================================== */

/**
 * @brief Reset handler called on system reset.
 *
 * This function performs the minimal C runtime initialization required by
 * C programs:
 *  1. Copy the initialized data from flash (_sidata) to RAM (.data)
 *  2. Zero the .bss section in RAM
 *  3. Call main()
 *
 * @note If main() ever returns, the handler loops forever. On embedded
 *       systems it is common to not expect main() to return.
 */
void Reset_Handler(void) {
  uint32_t *src, *dst;

  /* Copy the .data section (initialized global/static variables) from its
   * location in flash (where the compiler/linker stores the initial values)
   * to RAM (where they will actually live during execution). */
  src = &_sidata;
  dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  /* Zero out the .bss section (uninitialized global/static variables). */
  dst = &_sbss;
  while (dst < &_ebss) {
    *dst++ = 0;
  }

  /* Jump into main() - application entry point */
  main();

  /* If main() returns, stay here. Use a breakpoint to detect this during
   * debugging. */
  while (1)
    ;
}

/* ===================================================================== */
/* Default Handler                                                         */
/* ===================================================================== */

/**
 * @brief Default interrupt handler.
 *
 * All unimplemented exceptions/IRQs are aliased to this handler by default.
 * The handler simply loops indefinitely. During development it is useful to
 * set a breakpoint here to detect unexpected interrupts.
 */
void Default_Handler(void) {
  /* Hang here - Set a breakpoint to catch unexpected interrupts */
  while (1)
    ;
}

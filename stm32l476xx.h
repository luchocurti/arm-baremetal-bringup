/*!
 * @file    stm32l476xx_commented.h
 * @brief   Minimal STM32L476xx device header with extended register definitions
 *
 * A deliberately small subset of the full ST CMSIS / HAL header for the
 * STM32L4 series. This file contains core Cortex-M4 peripherals and a
 * selection of device peripherals (RCC, USART2, GPIOA) required by small
 * bare-metal projects or examples. It is NOT a replacement for the official
 * ST headers; use those for production projects.
 */

#ifndef STM32L476XX_H
#define STM32L476XX_H

#include <stdint.h>

/* Convenience qualifiers -------------------------------------------------- */
#ifndef __IO
/** @brief Read/Write volatile qualifier for memory-mapped registers. */
#define __IO volatile
#endif

/* --------------------------------------------------------------------------
 * Core Peripherals (ARM Cortex-M4)
 * -------------------------------------------------------------------------- */

/* --- SysTick Timer ------------------------------------------------------- */
/** @brief SysTick base address */
#define SysTick_BASE (0xE000E010UL)

/** @brief SysTick register layout (see Cortex-M4 Generic User Guide) */
typedef struct {
  __IO uint32_t CTRL; /**< 0x000: Control and Status Register */
  __IO uint32_t
      LOAD; /**< 0x004: Reload Value Register (24-bit value used by counter) */
  __IO uint32_t
      VAL; /**< 0x008: Current Value Register (reads the current counter) */
  __IO uint32_t
      CALIB; /**< 0x00C: Calibration Value Register (implementation-specific) */
} SysTick_Type;

/** @brief Pointer to the SysTick registers */
#define SysTick ((SysTick_Type *)SysTick_BASE)

/* SysTick CTRL register bit masks (use these to configure the timer) */
#define SysTick_CTRL_ENABLE_Msk (1UL << 0) /**< Counter enable */
#define SysTick_CTRL_TICKINT_Msk                                               \
  (1UL << 1) /**< SysTick exception request (interrupt) enable */
#define SysTick_CTRL_CLKSOURCE_Msk (1UL << 2) /**< Clock source selection */

/* --- System Control Block (SCB) ----------------------------------------- */
/** @brief SCB base address */
#define SCB_BASE (0xE000ED00UL)

/**
 * @brief System Control Block (SCB) register layout
 *
 * Contains core registers used for exception control, vector table offset,
 * CPU identification and system handlers. See ARMv7-M Architecture
 * documentation for details.
 */
typedef struct {
  __IO uint32_t CPUID; /**< 0x000: CPU ID Base Register */
  __IO uint32_t ICSR;  /**< 0x004: Interrupt Control and State Register */
  __IO uint32_t VTOR;  /**< 0x008: Vector Table Offset Register */
  __IO uint32_t
      AIRCR; /**< 0x00C: Application Interrupt and Reset Control Register */
  __IO uint32_t SCR; /**< 0x010: System Control Register */
  __IO uint32_t CCR; /**< 0x014: Configuration and Control Register */
  __IO uint8_t
      SHP[12]; /**< 0x018: System Handler Priority Registers (8-bit each) */
  __IO uint32_t SHCSR; /**< 0x024: System Handler Control and State Register */
} SCB_Type;

/** @brief Pointer to SCB registers */
#define SCB ((SCB_Type *)SCB_BASE)

/* SCB->SCR bit positions (use with SCB->SCR) */
#define SCB_SCR_SLEEPONEXIT_Pos 1U
#define SCB_SCR_SLEEPDEEP_Pos 2U
#define SCB_SCR_SLEEPONEXIT_Msk (1UL << SCB_SCR_SLEEPONEXIT_Pos)

/* --- Nested Vectored Interrupt Controller (NVIC) ------------------------ */
/** @brief NVIC base address */
#define NVIC_BASE (0xE000E100UL)
/** @brief NVIC IRQ number for USART2 (per STM32L476 reference manual). */
#define NVIC_IRQ_USART2 38U

/**
 * @brief NVIC register layout (simplified view)
 *
 * The NVIC provides registers to enable/disable interrupts, set/clear
 * pending state, and priority settings. ISER/ICER/ISPR/ICPR are arrays of
 * 32-bit registers; IP contains 8-bit priority fields.
 */
typedef struct {
  __IO uint32_t ISER[8];  /**< 0x000: Interrupt Set-Enable Registers */
  uint32_t RESERVED0[24]; /**< 0x020 - 0x07C: reserved */
  __IO uint32_t ICER[8];  /**< 0x080: Interrupt Clear-Enable Registers */
  uint32_t RESERVED1[24];
  __IO uint32_t ISPR[8]; /**< 0x100: Interrupt Set-Pending Registers */
  uint32_t RESERVED2[24];
  __IO uint32_t ICPR[8]; /**< 0x180: Interrupt Clear-Pending Registers */
  uint32_t RESERVED3[24];
  __IO uint32_t IABR[8]; /**< 0x200: Interrupt Active Bit Registers */
  uint32_t RESERVED4[56];
  __IO uint8_t IP[240]; /**< 0x300: Interrupt Priority Registers (8-bit each) */
  uint32_t RESERVED5[644]; /**< 0x3F0 - 0xEFC: reserved */
  __IO uint32_t STIR;      /**< 0xF00: Software Trigger Interrupt Register */
} NVIC_Type;

/** @brief Pointer to NVIC registers */
#define NVIC ((NVIC_Type *)NVIC_BASE)

/* --------------------------------------------------------------------------
 * Device-Specific Peripherals (STM32L4xx family)
 * -------------------------------------------------------------------------- */

/* --- Reset and Clock Control (RCC) -------------------------------------- */
/** @brief RCC base address (Reset and Clock Control) */
#define RCC_BASE (0x40021000UL)

/**
 * @brief Partial RCC register map
 *
 * This structure provides the fields required by common clock setup and
 * peripheral enabling routines. The order and reserved fields follow the
 * device reference manual. Only a subset of registers are present.
 */
typedef struct {
  volatile uint32_t CR;      /**< 0x00: Control register */
  volatile uint32_t ICSCR;   /**< 0x04: Internal Clock Sources Calibration */
  volatile uint32_t CFGR;    /**< 0x08: Clock configuration register */
  volatile uint32_t PLLCFGR; /**< 0x0C: PLL configuration register */
  volatile uint32_t PLLSAI1CFGR; /**< 0x10: PLL SAI1 config */
  volatile uint32_t PLLSAI2CFGR; /**< 0x14: PLL SAI2 config */
  volatile uint32_t CIER;        /**< 0x18: Clock interrupt enable */
  volatile uint32_t CIFR;        /**< 0x1C: Clock interrupt flag */
  volatile uint32_t CICR;        /**< 0x20: Clock interrupt clear */
  volatile uint32_t RESERVED0;   /**< 0x24: reserved */
  volatile uint32_t AHB1RSTR;    /**< 0x28: AHB1 peripheral reset */
  volatile uint32_t AHB2RSTR;    /**< 0x2C: AHB2 peripheral reset */
  volatile uint32_t AHB3RSTR;    /**< 0x30: AHB3 peripheral reset */
  uint32_t RESERVED1;            /**< 0x34: reserved */
  volatile uint32_t APB1RSTR1;   /**< 0x38: APB1 reset register 1 */
  volatile uint32_t APB1RSTR2;   /**< 0x3C: APB1 reset register 2 */
  volatile uint32_t APB2RSTR;    /**< 0x40: APB2 reset register */
  uint32_t RESERVED2;            /**< 0x44: reserved */
  volatile uint32_t AHB1ENR;     /**< 0x48: AHB1 peripheral clock enable */
  volatile uint32_t AHB2ENR;     /**< 0x4C: AHB2 peripheral clock enable */
  volatile uint32_t AHB3ENR;     /**< 0x50: AHB3 peripheral clock enable */
  uint32_t RESERVED3;            /**< 0x54: reserved */
  volatile uint32_t
      APB1ENR1; /**< 0x58: APB1 peripheral clock enable register 1 */
  volatile uint32_t
      APB1ENR2; /**< 0x5C: APB1 peripheral clock enable register 2 */
  volatile uint32_t APB2ENR;    /**< 0x60: APB2 peripheral clock enable */
  uint32_t RESERVED4;           /**< 0x64: reserved */
  volatile uint32_t AHB1SMENR;  /**< 0x68: AHB1 clock enable in sleep mode */
  volatile uint32_t AHB2SMENR;  /**< 0x6C: AHB2 clock enable in sleep mode */
  volatile uint32_t AHB3SMENR;  /**< 0x70: AHB3 clock enable in sleep mode */
  uint32_t RESERVED5;           /**< 0x74: reserved */
  volatile uint32_t APB1SMENR1; /**< 0x78: APB1 clock enable in sleep mode 1 */
  volatile uint32_t APB1SMENR2; /**< 0x7C: APB1 clock enable in sleep mode 2 */
  volatile uint32_t APB2SMENR;  /**< 0x80: APB2 clock enable in sleep mode */
  uint32_t RESERVED6;           /**< 0x84: reserved */
  volatile uint32_t
      CCIPR; /**< 0x88: Peripherals independent clock configuration */
  volatile uint32_t RESERVED7; /**< 0x8C: reserved */
  volatile uint32_t BDCR;      /**< 0x90: Backup domain control register */
  volatile uint32_t CSR;       /**< 0x94: Clock control & status register */
  volatile uint32_t CRRCR;     /**< 0x98: Clock recovery RC register */
  volatile uint32_t CCIPR2;    /**< 0x9C: Additional CCIPR (if present) */
} RCC_TypeDef;

/** @brief Pointer to RCC registers */
#define RCC ((RCC_TypeDef *)RCC_BASE)

/* -------------------- RCC_AHB2ENR bits -------------------- */
/** @name RCC_AHB2ENR Bits
 *  Bits used to enable peripheral clocks on AHB2 bus. Use these to enable
 *  the GPIO ports and other AHB2 peripherals before accessing them.
 */
/**@{*/
#define RCC_AHB2ENR_GPIOAEN_Pos 0U
#define RCC_AHB2ENR_GPIOAEN_Msk (1UL << RCC_AHB2ENR_GPIOAEN_Pos)
/** @brief Bit mask to enable clock for GPIOA */
#define RCC_AHB2ENR_GPIOAEN RCC_AHB2ENR_GPIOAEN_Msk
/**@}*/

/* -------------------- RCC_APB1ENR1 bits -------------------- */
/** @name RCC_APB1ENR1 Bits
 *  Bits used to enable APB1 peripherals (low-speed) such as USART2.
 */
/**@{*/
#define RCC_APB1ENR1_USART2EN_Pos 17U
#define RCC_APB1ENR1_USART2EN_Msk (1UL << RCC_APB1ENR1_USART2EN_Pos)
/** @brief Bit mask to enable USART2 clock on APB1 bus */
#define RCC_APB1ENR1_USART2EN RCC_APB1ENR1_USART2EN_Msk
/**@}*/

/* -------------------- RCC_CR bits -------------------- */
#define RCC_CR_MSION_Pos 0U
#define RCC_CR_MSION_Msk (1UL << RCC_CR_MSION_Pos)
#define RCC_CR_MSION RCC_CR_MSION_Msk /**< MSI enable */
#define RCC_CR_MSIRDY_Pos 1U
#define RCC_CR_MSIRDY_Msk (1UL << RCC_CR_MSIRDY_Pos)
#define RCC_CR_MSIRDY RCC_CR_MSIRDY_Msk /**< MSI ready flag */
#define RCC_CR_MSIRGSEL (1U << 3)       /**< MSI range selection control bit */

/* RCC_CFGR SW bits (system clock switch) */
#define RCC_CFGR_SW_Pos 0U
#define RCC_CFGR_SW_Msk (0x3U << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW RCC_CFGR_SW_Msk
#define RCC_CFGR_SW_MSI                                                        \
  (0x0U << RCC_CFGR_SW_Pos) /**< Select MSI as system clock */

/* RCC_CFGR SWS bits (system clock switch status) */
#define RCC_CFGR_SWS_Pos 2U
#define RCC_CFGR_SWS_Msk (0x3U << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS RCC_CFGR_SWS_Msk
#define RCC_CFGR_SWS_MSI                                                       \
  (0x0U << RCC_CFGR_SWS_Pos) /**< MSI is used as system clock */

/* AHB and APB prescaler masks (CFGR) */
#define RCC_CFGR_HPRE_Pos (4U)
#define RCC_CFGR_HPRE_Msk                                                      \
  (0xFU << RCC_CFGR_HPRE_Pos) /**< AHB prescaler mask                          \
                               */
#define RCC_CFGR_PPRE1_Pos (10U)
#define RCC_CFGR_PPRE1_Msk                                                     \
  (0x7U << RCC_CFGR_PPRE1_Pos) /**< APB1 prescaler mask */
#define RCC_CFGR_PPRE2_Pos (13U)
#define RCC_CFGR_PPRE2_Msk                                                     \
  (0x7U << RCC_CFGR_PPRE2_Pos) /**< APB2 prescaler mask */

/* -------------------- RCC_ICSCR bits -------------------- */
#define RCC_ICSCR_MSIRANGE_Pos 4U
#define RCC_ICSCR_MSIRANGE_Msk (0xFU << RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE                                                     \
  RCC_ICSCR_MSIRANGE_Msk /**< MSI range (frequency selection) */

#define RCC_ICSCR_MSITRIM_Pos                                                  \
  16U /**< MSI trimming value position (factory or software trimming) */
#define RCC_ICSCR_MSITRIM_Msk (0xFFU << RCC_ICSCR_MSITRIM_Pos)

#define RCC_ICSCR_MSICAL_Pos 8U /**< Factory calibration value position */
#define RCC_ICSCR_MSICAL_Msk (0xFFU << RCC_ICSCR_MSICAL_Pos)

#define RCC_CCIPR_USART2SEL_Pos 2U
#define RCC_CCIPR_USART2SEL_Msk                                                \
  (0x3U << RCC_CCIPR_USART2SEL_Pos) /**< USART2 kernel clock selection */

/* MSI frequency range enumerations (values used with RCC->ICSCR MSIRANGE field)
 * These approximate the MSI internal RC oscillator frequencies defined by ST.
 */
#define RCC_MSIRANGE_0 0U   /**< ~100 kHz  */
#define RCC_MSIRANGE_1 1U   /**< ~200 kHz  */
#define RCC_MSIRANGE_2 2U   /**< ~400 kHz  */
#define RCC_MSIRANGE_3 3U   /**< ~800 kHz  */
#define RCC_MSIRANGE_4 4U   /**< ~1   MHz  */
#define RCC_MSIRANGE_5 5U   /**< ~2   MHz  */
#define RCC_MSIRANGE_6 6U   /**< ~4   MHz  */
#define RCC_MSIRANGE_7 7U   /**< ~8   MHz  */
#define RCC_MSIRANGE_8 8U   /**< ~16  MHz  */
#define RCC_MSIRANGE_9 9U   /**< ~24  MHz  */
#define RCC_MSIRANGE_10 10U /**< ~32  MHz  */
#define RCC_MSIRANGE_11 11U /**< ~48  MHz  */

/* ==================== USART ============================================= */

#define USART2_BASE (0x40004400UL) /**< USART2 base address */
#define USART2                                                                 \
  ((USART_TypeDef *)USART2_BASE) /**< Pointer to USART2 registers */

/** @brief USART register layout (partial) */
typedef struct {
  volatile uint32_t CR1;  /**< Control register 1 */
  volatile uint32_t CR2;  /**< Control register 2 */
  volatile uint32_t CR3;  /**< Control register 3 */
  volatile uint32_t BRR;  /**< Baud rate register */
  volatile uint32_t GTPR; /**< Guard time & prescaler (smartcard) */
  volatile uint32_t RTOR; /**< Receiver timeout register */
  volatile uint32_t RQR;  /**< Request register */
  volatile uint32_t ISR;  /**< Interrupt & status register */
  volatile uint32_t ICR;  /**< Interrupt flag clear register */
  volatile uint32_t RDR;  /**< Receive data register */
  volatile uint32_t TDR;  /**< Transmit data register */
} USART_TypeDef;

/* USART CR1 control bits (used to enable USART and its TX/RX) */
#define USART_CR1_UE (1U << 0) /**< USART enable */
#define USART_CR1_RE (1U << 2) /**< Receiver enable */
#define USART_CR1_TE (1U << 3) /**< Transmitter enable */
#define USART_CR1_TXEIE                                                        \
  (1U << 7) /**< TXE (data register empty) interrupt enable */
#define USART_CR1_RXNEIE                                                       \
  (1U << 5) /**< RXNE (read data ready) interrupt enable */

/* USART Interrupt & Status Register (ISR) flags */
#define USART_ISR_PE (1U << 0)   /**< Parity error */
#define USART_ISR_FE (1U << 1)   /**< Framing error */
#define USART_ISR_NE (1U << 2)   /**< Noise detected on line */
#define USART_ISR_ORE (1U << 3)  /**< Overrun error */
#define USART_ISR_RXNE (1U << 5) /**< Read data register not empty */
#define USART_ISR_TC (1U << 6)   /**< Transmission complete */
#define USART_ISR_TXE (1U << 7)  /**< Transmit data register empty */

/* USART Interrupt Flag Clear Register (ICR) bits (write 1 to clear) */
#define USART_ICR_PECF (1U << 0)   /**< Clear parity error flag */
#define USART_ICR_FECF (1U << 1)   /**< Clear framing error flag */
#define USART_ICR_NECF (1U << 2)   /**< Clear noise error flag */
#define USART_ICR_ORECF (1U << 3)  /**< Clear overrun error flag */
#define USART_ICR_IDLECF (1U << 4) /**< Clear idle line detected flag */
#define USART_ICR_TCCF (1U << 6)   /**< Clear transmission complete flag */

/* --- General-Purpose I/O (GPIOA example) -------------------------------- */
#define GPIOA_BASE (0x48000000UL) /**< GPIOA base address */

/** @brief GPIO register layout (common for STM32L4 series) */
typedef struct {
  __IO uint32_t MODER; /**< 0x00: Port mode register (2 bits per pin) */
  __IO uint32_t
      OTYPER; /**< 0x04: Output type register (push-pull/open-drain) */
  __IO uint32_t OSPEEDR; /**< 0x08: Output speed register (2 bits per pin) */
  __IO uint32_t PUPDR; /**< 0x0C: Pull-up/pull-down register (2 bits per pin) */
  __IO uint32_t IDR;   /**< 0x10: Input data register (read pin states) */
  __IO uint32_t ODR;   /**< 0x14: Output data register (set/clear outputs) */
  __IO uint32_t BSRR;  /**< 0x18: Bit set/reset register (atomic set/reset) */
  __IO uint32_t LCKR;  /**< 0x1C: Configuration lock register */
  __IO uint32_t AFR[2]; /**< 0x20/0x24: Alternate function low/high registers (4
                           bits/pin) */
} GPIO_TypeDef;

/** @brief Pointer to GPIOA registers */
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)

/** @brief Helper macro to generate a pin mask for pin number n (0..15) */
#define PIN(n) (1U << (n))

#endif /* STM32L476XX_H */

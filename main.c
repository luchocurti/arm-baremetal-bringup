/* main.c - Main application for STM32L476: Blink + USART echo
 *
 * @file main.c
 * @brief Blink onboard LED (LD2) and provide non-blocking USART2 echo.
 *
 * This file contains a small bare-metal application for the STM32L476 MCU.
 * It toggles the onboard LED (PA5) every 500 ms using SysTick and provides
 * a non-blocking USART2 echo service at 115200 baud using simple ring buffers
 * and interrupts. The CPU executes `wfi` (wait for interrupt) in the main
 * loop to reduce power consumption when idle.
 *
 * Design / assumptions:
 *  - Core clock is configured to 4 MHz (MSI); SysTick uses this frequency to
 *    generate 1 ms ticks. The systick_init() uses the constant 4000 derived
 *    from that assumption: (4_000_000 / 1000) = 4000.
 *  - USART2 TX is PA2 and RX is PA3 using AF7 (USART2).
 *  - Simple single-producer / single-consumer ring buffers are used for TX
 *    and RX. The ISR (USART2 IRQ) is the consumer of the TX buffer and the
 *    producer of the RX buffer. The main context acts as the producer for TX
 *    and consumer for RX.
 *  - Buffer sizes are powers-of-two friendly (here 256) to keep index math
 *    simple, but code uses modulo to be explicit and portable.
 *
 * @author Luciano Curti
 * @version 1.0
 * @date 2025-08-25
 *
 * @note This file uses direct register access (CMSIS-style headers).
 *       Ensure the included device header (stm32l476xx.h) matches the MCU.
 */

#include "stm32l476xx.h"

/* -------------------------------------------------------------------------- */
/* Constants / Macros */
/* -------------------------------------------------------------------------- */

/** LD2 is connected to PA5 on the Nucleo board. Use PIN macro to compute bit.
 */
#define LED_PIN PIN(5) /* LD2 is on PA5 */
#define LED_PORT GPIOA

/**
 * USART port used for TX/RX (PA2/PA3)
 */
#define USART_PORT GPIOA

/** Size of the USART TX ring buffer (bytes). */
#define USART_TX_BUF_SIZE 256

/** Size of the USART RX ring buffer (bytes). */
#define USART_RX_BUF_SIZE 256

/* -------------------------------------------------------------------------- */
/* Globals */
/* -------------------------------------------------------------------------- */

/**
 * @brief Global millisecond counter incremented from SysTick_Handler.
 */
volatile uint32_t ms_ticks = 0;

/* -------------------------------------------------------------------------- */
/* USART ring buffers and indices (single-producer / single-consumer)
 *
 * Access patterns:
 *  - usart_rx_buf: written by ISR (usart_rx_push), read by main via
 *    usart_getc_nonblocking.
 *  - usart_tx_buf: written by main via
 * usart_putc_nonblocking/usart_tx_nonblocking, consumed by ISR in
 * USART2_IRQHandler.
 */
static volatile char usart_tx_buf[USART_TX_BUF_SIZE];
static volatile uint8_t usart_tx_head = 0; /* Producer index (main thread) */
static volatile uint8_t usart_tx_tail = 0; /* Consumer index (ISR) */
static volatile char usart_rx_buf[USART_RX_BUF_SIZE];
static volatile uint8_t usart_rx_head = 0; /* Producer index (ISR) */
static volatile uint8_t usart_rx_tail = 0; /* Consumer index (main thread) */

/* -------------------------------------------------------------------------- */
/* USART RX helper - called from ISR */
/* -------------------------------------------------------------------------- */
/**
 * @brief Push a received character into the RX ring buffer.
 *
 * This function is intended to be called from the USART RX interrupt
 * (ISR). If the buffer is full, the incoming character is dropped to avoid
 * blocking the ISR.
 *
 * @param c Character received from USART (8-bit ASCII).
 */
static void usart_rx_push(char c) {
  uint8_t next_head = (usart_rx_head + 1) % USART_RX_BUF_SIZE;
  if (next_head != usart_rx_tail) { /* Drop if buffer full */
    usart_rx_buf[usart_rx_head] = c;
    usart_rx_head = next_head;
  }
}

/* -------------------------------------------------------------------------- */
/* USART RX consumer - non-blocking */
/* -------------------------------------------------------------------------- */
/**
 * @brief Non-blocking read of a character from the RX buffer.
 *
 * This function does not disable interrupts; it assumes the single-producer
 * (ISR) / single-consumer (main) rule. Returns immediately if there is no
 * data available.
 *
 * @param[out] c Pointer to a char where the received byte will be stored if
 *               available.
 * @return int 1 if a character was returned (stored into *c), 0 if buffer
 *             was empty.
 */
int usart_getc_nonblocking(char *c) {
  if (usart_rx_head == usart_rx_tail) {
    /* Empty */
    return 0;
  }
  *c = usart_rx_buf[usart_rx_tail];
  usart_rx_tail = (usart_rx_tail + 1) % USART_RX_BUF_SIZE;
  /* Success */
  return 1;
}

/* -------------------------------------------------------------------------- */
/* USART TX producer - non-blocking */
/* -------------------------------------------------------------------------- */
/**
 * @brief Put a character into the TX ring buffer (non-blocking).
 *
 * The function enables the TXE interrupt so the ISR can consume the
 * buffer. If the buffer is full the character is dropped and -1 is returned.
 *
 * @param c Character to send.
 * @return int 0 on success, -1 if the TX buffer is full.
 */
static int usart_putc_nonblocking(char c) {
  uint8_t next_head = (usart_tx_head + 1) % USART_TX_BUF_SIZE;
  if (next_head == usart_tx_tail) {
    /* Buffer full. Report the error: drop char or block */
    return -1;
  }
  usart_tx_buf[usart_tx_head] = c;
  usart_tx_head = next_head;
  /* Enable TXE interrupt so ISR will start sending bytes from buffer. */
  USART2->CR1 |= USART_CR1_TXEIE;

  return 0;
}

/**
 * @brief Send a null-terminated string via the non-blocking TX buffer.
 *
 * Iterates over the string placing characters into the TX buffer until either
 * the string terminator is reached or the buffer becomes full.
 *
 * @param s Null-terminated C string to transmit.
 * @return int 0 on success (all characters queued), -1 if the buffer became
 *             full before all characters could be queued.
 */
static int usart_tx_nonblocking(const char *s) {
  int result = 0;
  while ((*s) && (result == 0)) {
    result = usart_putc_nonblocking(*s++);
  }

  return result;
}

/* -------------------------------------------------------------------------- */
/* USART2 Initialization */
/* -------------------------------------------------------------------------- */
/**
 * @brief Initialize USART2 for 8N1, configured on PA2 (TX) / PA3 (RX).
 *
 * This routine configures the GPIO alternate function, enables the peripheral
 * clock and configures USART2 registers. Baud rate calculation assumes a
 * peripheral clock of 4 MHz and uses simple integer division: BRR = PCLK /
 * baud. For production code, consider using oversampling and more precise BRR
 * calculations if necessary.
 *
 * @param baud Desired baud rate (e.g. 115200).
 */
void usart2_init(uint32_t baud) {
  /* Configure PA2 (TX) and PA3 (RX) for alternate function 7 (USART2) */
  USART_PORT->MODER &=
      ~((3U << (2 * 2)) | (3U << (3 * 2))); /* Clear mode bits */
  USART_PORT->MODER |=
      ((2U << (2 * 2)) | (2U << (3 * 2))); /* Alternate function mode */
  USART_PORT->AFR[0] &= ~((0xFU << (4 * 2)) | (0xFU << (4 * 3)));
  USART_PORT->AFR[0] |= ((7U << (4 * 2)) | (7U << (4 * 3)));  /* AF7 = USART2 */
  USART_PORT->OSPEEDR |= ((3U << (2 * 2)) | (3U << (3 * 2))); /* High speed */
  USART_PORT->OTYPER &= ~((1U << 2) | (1U << 3));             /* Push-pull */
  USART_PORT->PUPDR &=
      ~((3U << (2 * 2)) | (3U << (3 * 2))); /* No pull-up/pull-down */

  /* Enable USART2 clock */
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

  /* Configure USART2 */
  USART2->CR1 = 0;                /* Disable USART before config */
  USART2->BRR = 4000000UL / baud; /* Baud rate @ 4 MHz */
  /* Clear any old error flags by writing to ICR. */
  USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF |
                USART_ICR_PECF | USART_ICR_IDLECF |
                USART_ICR_TCCF;    /* Clear any old flags */
  USART2->CR1 |= USART_CR1_TE;     /* Enable transmitter */
  USART2->CR1 |= USART_CR1_RE;     /* Receiver enable */
  USART2->CR1 |= USART_CR1_RXNEIE; /* RX not empty interrupt */
  USART2->CR1 |= USART_CR1_UE;     /* Enable USART */

  /* Enable USART2 interrupt in NVIC (simple CMSIS-like approach). */
  NVIC->ISER[NVIC_IRQ_USART2 / 32] |= (1U << (NVIC_IRQ_USART2 % 32));

  /* Print an initial message */
  usart_tx_nonblocking("usart2_init\r\n");
}

/* -------------------------------------------------------------------------- */
/* USART2 IRQ Handler */
/* -------------------------------------------------------------------------- */
/**
 * @brief USART2 interrupt handler.
 *
 * Handles RXNE (receive) and TXE (transmit buffer empty) events. Also
 * clears error conditions (overrun, framing, noise, parity) by reading the
 * RDR and writing the ICR as required by the reference manual.
 *
 * Notes:
 *  - This handler should be kept short to avoid delaying other interrupts.
 *  - Reading USART2->RDR clears the RXNE flag.
 */
void USART2_IRQHandler(void) {
  uint32_t isr = USART2->ISR;

  /* Clear error conditions (read RDR, then write ICR) */
  if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
    volatile uint32_t discard = USART2->RDR;
    (void)discard;
    USART2->ICR =
        USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
  }

  /* RXNE flag = data received */
  if (isr & USART_ISR_RXNE) {
    char c = (char)(USART2->RDR & 0xFF); /* Reading clears RXNE */
    usart_rx_push(c);
  }

  /* TXE flag = transmit buffer empty */
  if ((isr & USART_ISR_TXE) && (usart_tx_head != usart_tx_tail)) {
    USART2->TDR = usart_tx_buf[usart_tx_tail];
    usart_tx_tail = (usart_tx_tail + 1) % USART_TX_BUF_SIZE;
  }

  /* If no more TX data, disable TXE interrupt to avoid continuous IRQs. */
  if (usart_tx_head == usart_tx_tail) {
    USART2->CR1 &= ~USART_CR1_TXEIE;
  }
}

/* -------------------------------------------------------------------------- */
/* SysTick Handler - millisecond tick                                         */
/* -------------------------------------------------------------------------- */
/**
 * @brief SysTick interrupt handler called every 1 ms.
 *
 * Increments the global millisecond counter and performs short periodic
 * tasks. Keep the handler as short as possible to minimize interrupt
 * latency for other peripherals.
 */
void SysTick_Handler(void) {
  ms_ticks++;

  /* Task A: Blink LED every 500 ms */
  static uint32_t last_led = 0;
  if (ms_ticks - last_led >= 500) {
    last_led = ms_ticks;
    LED_PORT->ODR ^= LED_PIN; /* Toggle LED */
  }

  /* Example Task B: USART output every 1000 ms (disabled by default)
   * If enabled, make sure it does not block or perform heavy work in ISR.
   */
  // Task B: USART output every 1000 ms
  // static uint32_t last_usart = 0;
  // if (ms_ticks - last_usart >= 1000) {
  //   last_usart = ms_ticks;
  //   int result = usart_tx_nonblocking("Task B\r\n");
  //   if (result != 0) {
  //     /* Error: TX buffer full */
  //   }
  // }
}

/* -------------------------------------------------------------------------- */
/* Clock configuration */
/* -------------------------------------------------------------------------- */
/**
 * @brief Configure MSI to 4 MHz and switch system clock to MSI.
 *
 * This function turns on the MSI oscillator and sets the MSIRANGE to the
 * register value for 4 MHz. It also switches the SYSCLK source to MSI and
 * configures prescalers to default (/1). This routine does minimal error
 * checking for clarity; production code may perform more robust checks.
 */
static void clock_init(void) {
  /* Make sure the MSI clock is ON */
  RCC->CR |= RCC_CR_MSION;
  while ((RCC->CR & (1U << 1)) == 0) {
  } /* MSIRDY bit1 */

  /* Use MSIRANGE from RCC_ICSCR instead of RCC_CR */
  RCC->CR |= RCC_CR_MSIRGSEL;

  /* Set the MSI frequency to 4 MHz (range 6) WITHOUT touching MSITRIM/others */
  uint32_t icscr = RCC->ICSCR;
  icscr &= ~RCC_ICSCR_MSIRANGE_Msk;
  icscr |= (RCC_MSIRANGE_6 << RCC_ICSCR_MSIRANGE_Pos);
  RCC->ICSCR = icscr;

  /* Ensure SYSCLK source is MSI and prescalers are /1 */
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW_Msk | RCC_CFGR_HPRE_Msk |
                             RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk)) |
              RCC_CFGR_SW_MSI;

  /* Wait for SWS = MSI */
  while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_MSI) {
  }
}

/* -------------------------------------------------------------------------- */
/* GPIO initialization */
/* -------------------------------------------------------------------------- */
/**
 * @brief Initialize GPIOA pins used for LED and USART.
 *
 * Enables the clock for GPIOA and configures PA5 as digital output for the
 * LED. USART2 pins (PA2/PA3) are configured in usart2_init(), not here.
 */
static void gpio_init(void) {
  /* Enable GPIOA clock */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  /* Set PA5 to output mode */
  LED_PORT->MODER &= ~(3U << (5 * 2));
  LED_PORT->MODER |= (1U << (5 * 2));

  // Optional: push-pull, no pull-up/down
  // LED_PORT->OTYPER &= ~(1UL << 5);
  // LED_PORT->PUPDR &= ~(3UL << (5 * 2));
}

/* -------------------------------------------------------------------------- */
/* SysTick setup */
/* -------------------------------------------------------------------------- */
/**
 * @brief Initialize SysTick to fire every 1 ms.
 *
 * Assumes the processor clock is 4 MHz. The LOAD value is computed as
 * (CORE_CLOCK_HZ / 1000) - 1 = 4000 - 1.
 */
void systick_init(void) {
  SysTick->LOAD = 4000 - 1;                    /* 4 MHz / 4000 = 1 ms */
  SysTick->VAL = 0;                            /* Clear current value */
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | /* Processor clock */
                  SysTick_CTRL_TICKINT_Msk |   /* Enable interrupt */
                  SysTick_CTRL_ENABLE_Msk;     /* Enable SysTick */
}

/* -------------------------------------------------------------------------- */
/* Main program */
/* -------------------------------------------------------------------------- */
/**
 * @brief Program entry point.
 *
 * Performs minimal system initialization and enters a low-power main loop
 * where the CPU executes WFI until an interrupt (SysTick or USART) wakes it.
 * The main loop polls the RX ring buffer and echoes received characters back
 * over USART2.
 *
 * @return int Never returns in this bare-metal example. Return value is
 *             provided to satisfy standard C signature for main().
 */
int main(void) {
  char c;

  /* System Startup: configure clock, GPIO, USART2, and SysTick */
  clock_init();
  gpio_init();
  usart2_init(115200);
  systick_init();

  /* Keep Sleep-on-exit OFF so main() continues running after exits from
   * interrupt handlers. If Sleep-on-exit is enabled, the core would return
   * to sleep automatically after ISRs, which is not the desired behavior
   * for this simple application. */
  SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

  /* Main loop does nothing — CPU sleeps until SysTick or USART interrupt
   * wakes it. Incoming bytes are read from the RX buffer and echoed back
   * non-blocking. */
  while (1) {
    /* Waits for input on USART2, echoes back characters. */
    if (usart_getc_nonblocking(&c)) {
      /* Non-blocking transmit: this may fail (return -1) if the TX buffer
       * is full. In that case the character is dropped. For a robust
       * application consider: retrying, blocking until space is available,
       * or flow control (RTS/CTS). */
      usart_putc_nonblocking(c);

      /* Move to the next line when CR received. This mirrors a typical
       * terminal behavior where CR is followed by LF. */
      if (c == '\r') {
        usart_putc_nonblocking('\n');
      }
    }

    /* Enter sleep (`wfi`) until next interrupt. Wake on SysTick or USART IRQ,
     * then return here and continue processing. */
    __asm volatile("wfi");
  }

  /* Should never be reached in a bare-metal app. */
  // return 0;
}

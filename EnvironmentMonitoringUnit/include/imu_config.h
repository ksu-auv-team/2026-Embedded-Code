#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/**
 * ============================================================================
 * BNO086 IMU CONFIGURATION
 *
 * All IMU-specific settings live here (separated from the general config.h):
 * UART mode, baud rate, clock source, and the USART/pins used for IF_IMU.
 *
 * Control pins are fixed by the board wiring (see src/bno086.cpp):
 *   NRST=PB4  BOOTN=PA1  H_INTN=PB5  CLKSEL0=PA15
 * ============================================================================
 */

/* ---------------------------------------------------------------------------
 * 1. UART MODE
 *
 * The BNO086 is strapped (PS0/PS1) for UART-SHTP - full Sensor Hub Transport
 * Protocol, bidirectional, with a command channel. This is the only mode
 * supported by the firmware: the IMU is driven by the imu_source driver, which
 * speaks SHTP wrapped in RFC1662/HDLC framing over the IMU UART.
 *
 * (The legacy one-way UART-RVC path and its hand-rolled parser were removed
 * when the driver switched to UART-SHTP.)
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * 2. BAUD RATE
 *
 * UART-SHTP on this board is strapped for 3 Mbaud. Override by defining
 * IMU_BAUD_RATE before including this header if your module differs.
 * ------------------------------------------------------------------------- */
#ifndef IMU_BAUD_RATE
  #define IMU_BAUD_RATE 3000000
#endif

/* ---------------------------------------------------------------------------
 * 3. CLOCK SOURCE (CLKSEL0, sampled at the rising edge of reset)
 *
 *   IMU_CLOCK_CRYSTAL  - CLKSEL0 LOW : on-board crystal / internal clock.
 *   IMU_CLOCK_EXTERNAL - CLKSEL0 HIGH: external clock signal on the CLK pin
 *                        (the CLK pin must be driven, typically 32.768 kHz).
 * ------------------------------------------------------------------------- */
#define IMU_CLOCK_CRYSTAL  0
#define IMU_CLOCK_EXTERNAL 1

#define IMU_CLOCK IMU_CLOCK_CRYSTAL

/* ---------------------------------------------------------------------------
 * 4. IMU USART SELECTION (peripheral + pins for IF_IMU)
 *
 * This 32-pin package only bonds out USART1/USART2/LPUART1. Options:
 *   IMU_SEL_USART1_PB6_PB7  - USART1  TX=PB6  RX=PB7   (default)
 *   IMU_SEL_USART1_PA9_PA10 - USART1  TX=PA9  RX=PA10
 *   IMU_SEL_USART2_PB3_PB4  - USART2  TX=PB3  RX=PB4   (conflicts BNO086 NRST!)
 *
 * Must not collide with the debug UART (UART_SELECT in config.h); enforced
 * there. PB4 also clashes with NRST, so the PB3/PB4 option is blocked.
 * ------------------------------------------------------------------------- */
#define IMU_SEL_USART1_PB6_PB7  0
#define IMU_SEL_USART1_PA9_PA10 1
#define IMU_SEL_USART2_PB3_PB4  2

#define IMU_UART_SELECT IMU_SEL_USART1_PB6_PB7

#endif // IMU_CONFIG_H

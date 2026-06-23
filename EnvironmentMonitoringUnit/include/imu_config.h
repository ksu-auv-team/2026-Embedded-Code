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
 * The BNO086 supports two UART protocols, strapped on the board via PS0/PS1.
 * This MUST match how your board is strapped.
 *
 *   IMU_MODE_SHTP - full Sensor Hub Transport Protocol. Bidirectional;
 *                   supports the "get info" Product ID Request. Runs at 3 Mbaud.
 *   IMU_MODE_RVC  - UART-RVC: one-way 100 Hz streaming of heading/accel only,
 *                   no command channel. Runs at 115200 baud.
 * ------------------------------------------------------------------------- */
#define IMU_MODE_SHTP 0
#define IMU_MODE_RVC  1

#define IMU_USART_MODE IMU_MODE_RVC

/* ---------------------------------------------------------------------------
 * 2. BAUD RATE
 *
 * The two modes have different fixed baud rates, so by default the baud is
 * derived from IMU_USART_MODE. To force a specific value (e.g. a board that
 * reconfigures the SHTP baud), define IMU_BAUD_RATE before this block.
 *   UART-SHTP -> 3,000,000   UART-RVC -> 115,200
 * ------------------------------------------------------------------------- */
#ifndef IMU_BAUD_RATE
  #if IMU_USART_MODE == IMU_MODE_SHTP
    #define IMU_BAUD_RATE 3000000
  #else
    #define IMU_BAUD_RATE 115200
  #endif
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

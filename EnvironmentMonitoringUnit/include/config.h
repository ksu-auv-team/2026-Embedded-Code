#ifndef CONFIG_H
#define CONFIG_H

/**
 * ============================================================================
 * USER CONFIGURATION - Single source of truth for the Environment Monitoring Unit.
 *
 * Target: STM32G431KBT6 (32-pin LQFP "K" package).
 *
 * The unit exposes three serial INTERFACES and can echo any of them to any
 * other via a routing table:
 *
 *   IF_USB  - native USB CDC        (PA11/PA12)
 *   IF_UART - debug UART            (selectable, default USART2 PA2/PA3 -> VCP)
 *   IF_IMU  - IMU UART              (USART1 PB6/PB7, permanently attached IMU)
 *
 * Customize behavior by editing the values below.
 * ============================================================================
 */

#include <Arduino.h>

/* Interface identifiers (used by the heartbeat and routing config). */
enum InterfaceId {
    IF_USB  = 0,
    IF_UART = 1,
    IF_IMU  = 2,
    IF_COUNT
};

/* ===========================================================================
 * 1. SYSTEM CLOCK
 *
 * The oscillator source (HSI/HSE) is fed through the PLL to produce SYSCLK -
 * the frequency the CPU and peripherals actually run at. Both the source and
 * the resulting SYSCLK are configurable here.
 *
 * --- 1a. Oscillator source ---
 * CLOCK_SOURCE_HSI - internal 16 MHz oscillator. No crystal needed. Default.
 * CLOCK_SOURCE_HSE - external crystal/oscillator (set HSE_FREQUENCY_HZ).
 *                    Better baud-rate accuracy than HSI.
 *
 * USB always sources its 48 MHz from HSI48 regardless, so HSE need not be a
 * USB-grade crystal. The tool always overrides the clock setup (so SYSCLK_HZ
 * below is honored on both HSI and HSE paths) - see src/clock_config.cpp.
 * ========================================================================= */
#define CLOCK_SOURCE_HSI 0
#define CLOCK_SOURCE_HSE 1

#define CLOCK_SOURCE CLOCK_SOURCE_HSI

/* External crystal frequency in Hz (only used when CLOCK_SOURCE == HSE).
 * Supported out of the box: 8, 16, or 24 MHz (PLLM cases in clock_config.cpp). */
#define HSE_FREQUENCY_HZ 16000000UL

/* --- 1b. Target SYSCLK (the speed instructions execute at) ---
 *
 * The source is divided to a 4 MHz PLL input, then multiplied/divided back up
 * to SYSCLK. Pick one of the suggested targets; each has verified PLL settings,
 * flash wait-states and voltage scaling in src/clock_config.cpp.
 *
 *   SYSCLK_170MHZ - maximum performance (chip max). Default.
 *   SYSCLK_144MHZ - slightly lower power / EMI, still fast.
 *   SYSCLK_128MHZ - lower power; drops a flash wait state.
 *   SYSCLK_64MHZ  - low power.
 *
 * (USB CDC is unaffected - it always runs from the separate 48 MHz HSI48.)
 */
#define SYSCLK_170MHZ 170000000UL
#define SYSCLK_144MHZ 144000000UL
#define SYSCLK_128MHZ 128000000UL
#define SYSCLK_64MHZ   64000000UL

#define SYSCLK_HZ SYSCLK_144MHZ

/* ===========================================================================
 * 2. DEBUG UART SELECTION (the physical USART used for IF_UART)
 *
 * Pick ONE. Each maps to a specific peripheral + pin pair on this package.
 * NOTE: the IMU permanently uses USART1 on PB6/PB7, so IF_UART must NOT be set
 * to USART1 (a compile-time check below enforces this).
 * ========================================================================= */
#define UART_SEL_USART2_PA2_PA3 0   /* USART2  TX=PA2  RX=PA3   (ST-Link VCP) */
#define UART_SEL_USART1_PB6_PB7 1   /* USART1  TX=PB6  RX=PB7   (conflicts IMU!) */
#define UART_SEL_LPUART1_PA2_PA3 2  /* LPUART1 TX=PA2  RX=PA3                 */

#define UART_SELECT UART_SEL_USART2_PA2_PA3

/* Baud rate for the debug UART (IF_UART). USB CDC ignores baud. */
#define BAUD_RATE 115200

/* ===========================================================================
 * 3. IMU (IF_IMU) - permanently attached BNO086
 *
 * All IMU settings (UART mode, baud, clock, USART/pins) live in their own file.
 * ========================================================================= */
#include "imu_config.h"

/* ===========================================================================
 * 4. ECHO ROUTING TABLE  (any interface -> any interface)
 *
 * Each entry forwards bytes received on 'from' out to 'to'. List as many
 * directed pairs as you like. Examples:
 *   { IF_IMU,  IF_USB }   - stream IMU data up to the USB host
 *   { IF_USB,  IF_IMU }   - send host commands down to the IMU
 *   { IF_UART, IF_USB }   - mirror the debug UART onto USB
 *
 * Leave the table empty ( {} with ROUTE_COUNT 0 ) to disable all echoing.
 * The table itself is defined in src/router.cpp's ROUTES[]; edit it there.
 * This section documents intent; see ROUTES[] for the live list.
 * ========================================================================= */

/* ===========================================================================
 * 5. HEARTBEAT
 *
 * The periodic counter. HEARTBEAT_DESTINATIONS is the list of interfaces that
 * receive it - any subset of InterfaceId. Default: USB only. Examples:
 *   #define HEARTBEAT_DESTINATIONS  IF_USB                 (default)
 *   #define HEARTBEAT_DESTINATIONS  IF_UART                debug UART only
 *   #define HEARTBEAT_DESTINATIONS  IF_USB, IF_UART        both
 * Leave it empty to disable the heartbeat:
 *   #define HEARTBEAT_DESTINATIONS  // (nothing)
 * ========================================================================= */
/* Native USB on this board is non-functional, so all debug output is routed to
 * the ST-Link VCP (IF_UART). */
#define HEARTBEAT_DESTINATIONS

#define SEND_INTERVAL_MS 1000
#define COUNTER_MAX 255

/* ===========================================================================
 * 6. LED INDICATOR (feature toggle)
 *
 * Set ENABLE_LED to 0 to compile the LED feature out entirely.
 * ========================================================================= */
#define ENABLE_LED 1
#define LED_PIN PA0
#define LED_PULSE_MS 100

/* ===========================================================================
 * 7. I2C SLAVE (IMU data output)
 *
 * The STM32 acts as an I2C SLAVE. An external host (the bus master) reads the
 * latest ImuPacket from the STM32 on demand - a plain I2C read returns the most
 * recent 14-byte packet. The STM32 never initiates transfers and does not set
 * the bus clock (the master drives SCL).
 *
 * Recommended free pins on the 32-pin K-package (no conflicts with USART1
 * PB6/PB7, USART2 PA2/PA3, USB PA11/PA12, or BNO086 control pins):
 *   I2C2  SDA=PA8  SCL=PA9   (default)
 *
 * I2C_SLAVE_ADDR - the 7-bit address this STM32 responds to.
 * ========================================================================= */
#define I2C_SLAVE_SDA   PA8
#define I2C_SLAVE_SCL   PA9
#define I2C_SLAVE_ADDR  0x4B

/* ===========================================================================
 * 8. IMU PUBLISH RATE
 *
 * The BNO086 streams at ~100 Hz, but publishing every frame floods the 115200
 * debug VCP and can block loop(). PUBLISH_RATE_HZ caps how often a frame is
 * published (to both the VCP line and the I2C bus); intermediate frames are
 * dropped. Set to 0 to publish every frame (no throttle).
 * ========================================================================= */
#define PUBLISH_RATE_HZ 20

/* ===========================================================================
 * DERIVED / GUARDS - do not edit below this line.
 * ========================================================================= */

/* USB_AVAILABLE reflects whether the USB CDC stack is compiled in. The single
 * build env (platformio.ini) always enables it. */
#if defined(USBCON) && defined(USBD_USE_CDC)
  #define USB_AVAILABLE 1
  #define USB_SERIAL SerialUSB   /* core's native USB CDC object */
#else
  #define USB_AVAILABLE 0
#endif

/* The debug UART (IF_UART) and the IMU UART (IF_IMU) must not share a
 * peripheral. Map each selection to a peripheral id, then compare. */
#if   UART_SELECT == UART_SEL_USART2_PA2_PA3
  #define DEBUG_UART_PERIPH 2   /* USART2  */
#elif UART_SELECT == UART_SEL_USART1_PB6_PB7
  #define DEBUG_UART_PERIPH 1   /* USART1  */
#elif UART_SELECT == UART_SEL_LPUART1_PA2_PA3
  #define DEBUG_UART_PERIPH 12  /* LPUART1 */
#endif

#if   IMU_UART_SELECT == IMU_SEL_USART1_PB6_PB7 || IMU_UART_SELECT == IMU_SEL_USART1_PA9_PA10
  #define IMU_UART_PERIPH 1     /* USART1  */
#elif IMU_UART_SELECT == IMU_SEL_USART2_PB3_PB4
  #define IMU_UART_PERIPH 2     /* USART2  */
#endif

#if DEBUG_UART_PERIPH == IMU_UART_PERIPH
  #error "UART_SELECT and IMU_UART_SELECT use the same peripheral. Choose different USARTs."
#endif

/* The IMU's USART2/PB3-PB4 option collides with the BNO086 NRST pin (PB4). */
#if IMU_UART_SELECT == IMU_SEL_USART2_PB3_PB4
  #error "IMU_SEL_USART2_PB3_PB4 conflicts with BNO086 NRST on PB4. Use USART1 pins."
#endif

#endif // CONFIG_H

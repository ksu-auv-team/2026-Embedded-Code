#ifndef BNO086_H
#define BNO086_H

/**
 * BNO086 IMU bring-up and "get basic info" over UART-SHTP.
 *
 * Control pins (this board):
 *   NRST    -> PB4    reset, active low (held low >=10 ms to reset)
 *   BOOTN   -> PA1    bootloader select; HIGH = normal operation
 *   H_INTN  -> PB5    host interrupt / data-ready, active LOW
 *   CLKSEL0 -> PA15   clock select; HIGH = external clock on the CLK pin
 *
 * BOOTN and CLKSEL0 are strapping pins sampled at the rising edge of reset, so
 * they are driven before NRST is released.
 *
 * Data path is the IMU UART (IF_IMU). The device must be strapped for
 * UART-SHTP mode (PS0/PS1) for the Product ID Request to work.
 *
 * bno086_begin() resets the device, waits for it to signal ready on H_INTN,
 * sends an SHTP Product ID Request, parses the Product ID Response, and prints
 * the decoded firmware/part/build info to the debug console (IF_UART).
 */

/** Reset the BNO086, query its product ID, and print decoded info. Blocking,
 *  with timeouts; safe to call once from setup() after interfaces_begin(). */
void bno086_begin(void);

#endif // BNO086_H

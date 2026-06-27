#ifndef BNO086_H
#define BNO086_H

/**
 * BNO086 physical bring-up (reset + strapping).
 *
 * Control pins (this board):
 *   NRST    -> PB4    reset, active low (held low >=10 ms to reset)
 *   BOOTN   -> PA1    bootloader select; HIGH = normal operation
 *   H_INTN  -> PB5    host interrupt / data-ready, active LOW
 *   CLKSEL0 -> PA15   clock select; HIGH = external clock on the CLK pin
 *
 * BOOTN and CLKSEL0 are strapping pins sampled at the rising edge of reset, so
 * they are driven before NRST is released. (PS0/PS1 are board-fixed for
 * UART-SHTP.)
 *
 * This module only resets and straps the chip. All SHTP traffic - Set Feature
 * and sensor reports - is handled by the imu_source driver; see imu_source.h.
 */

/** Reset and strap the BNO086 (NRST pulse with BOOTN/CLKSEL0 driven for
 *  UART-SHTP). Call once from setup() AFTER interfaces_begin() and BEFORE
 *  imu_source_setup(); the latter owns the UART and the post-reset settle. */
void bno086_reset(void);

#endif // BNO086_H

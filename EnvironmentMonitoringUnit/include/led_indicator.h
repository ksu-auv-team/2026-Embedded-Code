#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdint.h>

/**
 * Non-blocking LED indicator.
 *
 * Call led_pulse() to flash the LED; led_update() turns it back off after
 * LED_PULSE_MS without blocking the main loop. Currently pulsed on each I2C
 * read from the host (see data_publisher.cpp) as a bus-activity indicator.
 *
 * led_pulse() is safe to call from an ISR (it only does a digitalWrite + a
 * millis() read); led_update() runs in the main loop.
 *
 * When ENABLE_LED == 0 (see config.h) all three functions compile to no-ops.
 */

/** Configure the LED pin. Call once from setup(). */
void led_setup(void);

/** Blink the LED a few times at power-up (blocking). Call once from setup(),
 *  after led_setup(), as a boot/alive indicator. */
void led_startup_blink(void);

/** Hold the LED solid-on for 'ms' (blocking). Used at boot to confirm IMU data
 *  is being received. */
void led_solid(uint32_t ms);

/** Start an LED pulse (turn on; led_update() turns it off after LED_PULSE_MS). */
void led_pulse(void);

/** Non-blocking tick: turn the LED off once its pulse has elapsed. */
void led_update(void);

#endif // LED_INDICATOR_H

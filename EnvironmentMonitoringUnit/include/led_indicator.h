#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

/**
 * Non-blocking LED indicator.
 *
 * Call led_pulse() to flash the LED; led_update() turns it back off after
 * LED_PULSE_MS without blocking the main loop.
 *
 * When ENABLE_LED == 0 (see config.h) all three functions compile to no-ops.
 */

/** Configure the LED pin. Call once from setup(). */
void led_setup(void);

/** Start an LED pulse (turn on; led_update() turns it off after LED_PULSE_MS). */
void led_pulse(void);

/** Non-blocking tick: turn the LED off once its pulse has elapsed. */
void led_update(void);

#endif // LED_INDICATOR_H

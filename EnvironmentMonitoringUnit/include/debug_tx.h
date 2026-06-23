#ifndef DEBUG_TX_H
#define DEBUG_TX_H

/**
 * UART heartbeat transmitter.
 *
 * Every SEND_INTERVAL_MS, prints an incrementing counter (0..COUNTER_MAX,
 * wrapping) over DEBUG_SERIAL and triggers an LED pulse. Fully non-blocking.
 */

/** Print the boot banner. Call once from setup() after DEBUG_SERIAL.begin(). */
void debug_tx_setup(void);

/** Non-blocking tick: send the next counter value when the interval elapses. */
void debug_tx_update(void);

#endif // DEBUG_TX_H

#ifndef INTERFACES_H
#define INTERFACES_H

#include <Arduino.h>
#include "config.h"

/**
 * Serial interface registry.
 *
 * Unifies the three transports (USB CDC, debug UART, IMU UART) behind a common
 * Stream* handle so generic code (router, heartbeat) can read/write any of them
 * by InterfaceId without caring about the concrete type.
 *
 * HardwareSerial and USBSerial both derive from Stream, which is what makes the
 * uniform handle possible.
 */

/** Begin all interfaces at their configured baud rates. Call once from setup(). */
void interfaces_begin(void);

/**
 * Get the Stream for an interface id, or nullptr if that interface is not
 * available in this build (e.g. IF_USB on a non-USB build).
 */
Stream *interface_get(InterfaceId id);

/** Human-readable name for an interface id (for banners/logs). */
const char *interface_name(InterfaceId id);

#endif // INTERFACES_H

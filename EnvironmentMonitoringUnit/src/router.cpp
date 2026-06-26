#include "router.h"
#include "interfaces.h"

/* ===========================================================================
 * ECHO ROUTING TABLE - edit this list to control which interface echoes to
 * which. Each entry forwards bytes received on 'from' out to 'to'.
 *
 * IMU bytes are consumed by imu_reader (parsed into ImuPacket); do not add
 * IF_IMU as a source here or bytes will be split between the two consumers.
 *
 * Example routes you can add:
 *   { IF_UART, IF_USB },   mirror debug UART onto USB
 * ========================================================================= */
static const Route ROUTES[] = {
};

static const size_t ROUTE_COUNT = sizeof(ROUTES) / sizeof(ROUTES[0]);

/* Max bytes moved per route per tick - keeps one busy route from starving the
 * rest of loop() (and the other routes). */
static const int BYTES_PER_ROUTE_PER_TICK = 64;

void router_setup(void) {
    /* All interfaces are started in interfaces_begin(); nothing to do here. */
}

void router_update(void) {
    for (size_t i = 0; i < ROUTE_COUNT; i++) {
        Stream *src = interface_get(ROUTES[i].from);
        Stream *dst = interface_get(ROUTES[i].to);

        /* Skip routes whose endpoints aren't available in this build. */
        if (!src || !dst) continue;

        int budget = BYTES_PER_ROUTE_PER_TICK;
        while (src->available() && budget-- > 0) {
            dst->write((uint8_t)src->read());
        }
    }
}

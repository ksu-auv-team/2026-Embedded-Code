#include <Arduino.h>
#include "config.h"
#include "interfaces.h"
#include "led_indicator.h"
#include "debug_tx.h"
#include "router.h"
#include "bno086.h"

/**
 * STM32G431KBT6 Environment Monitoring Unit.
 *
 * Three serial interfaces (see config.h / interfaces.cpp):
 *   IF_USB  - native USB CDC      (PA11/PA12)
 *   IF_UART - debug UART          (selectable, default USART2 PA2/PA3 -> VCP)
 *   IF_IMU  - IMU UART            (USART1 PB6/PB7, permanently attached IMU)
 *
 * Features (each a module with setup() + non-blocking update()):
 *   - debug_tx : periodic counter heartbeat to configured interface(s)
 *   - router   : echo any interface to any interface per ROUTES[] (router.cpp)
 *   - led      : non-blocking LED pulse per heartbeat
 *
 * The main loop never blocks. Configure everything in include/config.h
 * (and the ROUTES[] / HEARTBEAT_DESTINATIONS[] tables in the .cpp files).
 */

void setup() {
    interfaces_begin();

#if USB_AVAILABLE
    /* Give the USB host up to 2 s to enumerate so the boot banner isn't missed,
     * but never block forever if nothing is attached. */
    uint32_t start = millis();
    while (!USB_SERIAL && (millis() - start < 2000)) {
        /* wait for host */
    }
#endif

    led_setup();
    router_setup();

    /* Reset the IMU and print its product ID before starting the heartbeat. */
    bno086_begin();

    debug_tx_setup();
}

void loop() {
    debug_tx_update();   // heartbeat to configured interface(s)
    router_update();     // echo interfaces per the routing table
    led_update();        // turn LED off when its pulse elapses
}

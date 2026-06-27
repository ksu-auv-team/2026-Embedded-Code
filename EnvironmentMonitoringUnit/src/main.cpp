#include <Arduino.h>
#include "config.h"
#include "interfaces.h"
#include "led_indicator.h"
#include "debug_tx.h"
#include "router.h"
#include "bno086.h"
#include "imu_source.h"
#include "data_publisher.h"

/**
 * STM32G431KBT6 Environment Monitoring Unit.
 *
 * Three serial interfaces (see config.h / interfaces.cpp):
 *   IF_USB  - native USB CDC      (PA11/PA12)
 *   IF_UART - debug UART          (selectable, default USART2 PA2/PA3 -> VCP)
 *   IF_IMU  - IMU UART            (USART1 PB6/PB7, permanently attached IMU)
 *
 * Data flow:
 *   BNO086 -> IF_IMU -> imu_source (7Semi BNO08x lib, UART-SHTP) -> data_publisher
 *     data_publisher -> USB serial monitor  (if host connected)
 *     data_publisher -> I2C output bus      (always, see config.h section 7)
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

    /* Bring up the debug heartbeat FIRST, so the VCP proves alive before the
     * (riskier) IMU bring-up. If IMU init ever hangs, the heartbeat still ran. */
    debug_tx_setup();

    /* Reset + strap the IMU before the library starts talking SHTP to it. */
    bno086_begin();

    imu_source_setup();
    data_publisher_setup();
}

void loop() {
    debug_tx_update();           // heartbeat to configured interface(s)
    router_update();             // echo interfaces per the routing table

    imu_source_update();         // read+decode SHTP reports from BNO086

    ImuPacket pkt;
    if (imu_source_get(pkt)) {
        data_publisher_publish(pkt); // -> VCP line + I2C (always)
    }

    led_update();                // turn LED off when its pulse elapses
}

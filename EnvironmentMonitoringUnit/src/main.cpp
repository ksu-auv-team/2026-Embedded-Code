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
 *   BNO086 -> IF_IMU -> imu_source (UART-SHTP driver) -> data_publisher
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
    led_startup_blink();         // power-up blink burst: board booted, LED works
    router_setup();

    /* Bring up the debug heartbeat FIRST, so the VCP proves alive before the
     * (riskier) IMU bring-up. If IMU init ever hangs, the heartbeat still ran. */
    debug_tx_setup();

    /* Reset + strap the IMU before imu_source starts talking SHTP to it. */
    bno086_reset();

    imu_source_setup();
    data_publisher_setup();

    /* Confirm the IMU is actually streaming: pump the reader briefly, and if a
     * packet arrives, hold the LED solid for a second as a boot-OK indicator. */
    uint32_t confirm_start = millis();
    bool imu_ok = false;
    ImuPacket boot_pkt;
    while (millis() - confirm_start < IMU_BOOT_CONFIRM_TIMEOUT_MS) {
        imu_source_update();
        if (imu_source_get(boot_pkt)) { imu_ok = true; break; }
    }
    if (imu_ok) led_solid(LED_STARTUP_SOLID_MS);
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

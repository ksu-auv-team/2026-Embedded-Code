#include "debug_tx.h"
#include "led_indicator.h"
#include "interfaces.h"
#include "config.h"

/* Destinations come from HEARTBEAT_DESTINATIONS in config.h. Build the list
 * here; an empty config macro yields an empty list (heartbeat disabled). */
static const InterfaceId heartbeat_dests[] = { HEARTBEAT_DESTINATIONS };

static const size_t HEARTBEAT_DEST_COUNT =
    sizeof(heartbeat_dests) / sizeof(InterfaceId);

/* Heartbeat state - private to this module. */
static uint8_t counter = 0;
static uint32_t last_send_ms = 0;

/* Emit one line (banner, or the counter when banner == nullptr) to every
 * configured, available heartbeat destination. */
static void heartbeat_print(const char *banner) {
    for (size_t i = 0; i < HEARTBEAT_DEST_COUNT; i++) {
        Stream *s = interface_get(heartbeat_dests[i]);
        if (!s) continue;  /* interface not available in this build */
        if (banner) s->println(banner);
        else        s->println(counter);
    }
}

void debug_tx_setup(void) {
    heartbeat_print("Environment Monitoring Unit Started");
}

void debug_tx_update(void) {
    if (millis() - last_send_ms < SEND_INTERVAL_MS) return;
    last_send_ms = millis();

    /* Transmit the current counter value to the configured destination(s). */
    heartbeat_print(nullptr);

    /* Flash the LED to show a transmission happened. */
    led_pulse();

    /* Advance the counter, wrapping at COUNTER_MAX. */
    if (counter >= COUNTER_MAX) {
        counter = 0;
    } else {
        counter++;
    }
}

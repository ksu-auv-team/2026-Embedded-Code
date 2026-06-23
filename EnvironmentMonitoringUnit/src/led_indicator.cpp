#include "led_indicator.h"
#include "config.h"

#if ENABLE_LED

/* Pulse timing state - private to this module. */
static uint32_t pulse_start_ms = 0;
static bool pulse_active = false;

void led_setup(void) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void led_pulse(void) {
    digitalWrite(LED_PIN, HIGH);
    pulse_start_ms = millis();
    pulse_active = true;
}

void led_update(void) {
    if (!pulse_active) return;

    if (millis() - pulse_start_ms >= LED_PULSE_MS) {
        digitalWrite(LED_PIN, LOW);
        pulse_active = false;
    }
}

#else  /* ENABLE_LED == 0 : feature compiled out */

void led_setup(void) {}
void led_pulse(void) {}
void led_update(void) {}

#endif // ENABLE_LED

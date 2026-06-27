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

void led_startup_blink(void) {
    /* Brief blocking blink burst at power-up: confirms the board booted and the
     * LED works, before the main loop / I2C activity takes over the LED. */
    for (uint8_t i = 0; i < LED_STARTUP_BLINKS; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(LED_STARTUP_BLINK_MS);
        digitalWrite(LED_PIN, LOW);
        delay(LED_STARTUP_BLINK_MS);
    }
}

void led_solid(uint32_t ms) {
    /* Hold the LED solid-on for 'ms' (blocking). Used at boot to confirm IMU
     * data is being received. */
    digitalWrite(LED_PIN, HIGH);
    delay(ms);
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
void led_startup_blink(void) {}
void led_solid(uint32_t ms) { (void)ms; }
void led_pulse(void) {}
void led_update(void) {}

#endif // ENABLE_LED

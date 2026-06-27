#include "bno086.h"
#include "interfaces.h"
#include "config.h"

/* --- Control pins (this board) --- */
#define BNO_NRST_PIN    PB4   /* reset, active low            */
#define BNO_BOOTN_PIN   PA1   /* HIGH = normal (non-DFU)      */
#define BNO_HINTN_PIN   PB5   /* host interrupt, active LOW   */
#define BNO_CLKSEL0_PIN PA15  /* clock select: HIGH = external clock on CLK pin */

/* --- Timing (from CEVA sh2 reference HAL) --- */
#define BNO_RESET_LOW_MS     12     /* hold NRST low >=10 ms                */

/* The debug console we print bring-up status to. */
static Stream *console(void) { return interface_get(IF_UART); }

void bno086_reset(void) {
    Stream *c = console();

    /* --- Physical bring-up ONLY: reset + strapping --- *
     * Strapping pins (BOOTN, CLKSEL0; PS0/PS1 are board-fixed for UART-SHTP)
     * are sampled at the RISING edge of reset, so they must be driven BEFORE
     * releasing NRST.
     *
     * This function deliberately does NOT touch the IMU UART - not begin(), not
     * read(). The imu_source driver (imu_source.cpp) is the SOLE owner of the
     * IMU UART: it begin()s the port and performs every read. An earlier
     * diagnostic "boot peek" here begin()'d and drained the UART, which left the
     * link in a half-initialized, mid-frame state and made the driver's first
     * Set-Feature exchange fail (reports never enabled). Removing it lets the
     * driver own a clean stream from reset.
     */
    pinMode(BNO_HINTN_PIN, INPUT_PULLUP);  /* active-low ready/interrupt */

    pinMode(BNO_NRST_PIN, OUTPUT);
    digitalWrite(BNO_NRST_PIN, LOW);       /* hold in reset while strapping */

    pinMode(BNO_BOOTN_PIN, OUTPUT);
    digitalWrite(BNO_BOOTN_PIN, HIGH);     /* normal operation, not DFU */
    pinMode(BNO_CLKSEL0_PIN, OUTPUT);
#if IMU_CLOCK == IMU_CLOCK_EXTERNAL
    digitalWrite(BNO_CLKSEL0_PIN, HIGH);   /* external clock on the CLK pin */
#else
    digitalWrite(BNO_CLKSEL0_PIN, LOW);    /* on-board crystal / internal clock */
#endif

    delay(BNO_RESET_LOW_MS);
    digitalWrite(BNO_NRST_PIN, HIGH);      /* release reset; straps sampled now */

    /* The post-reset settle (waiting for the boot advertisement before sending
     * commands) is owned by imu_source_setup(), the code that actually talks to
     * the chip - see BOOT_SETTLE_MS there. Do NOT read the UART here. */
    if (c) c->println("BNO086: reset + strapped (UART owned by imu_source)");
}

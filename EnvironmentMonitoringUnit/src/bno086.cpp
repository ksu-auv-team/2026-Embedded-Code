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
#define BNO_BOOT_SETTLE_MS   100    /* settle after reset before library talks */

/* The debug console we print bring-up status to. */
static Stream *console(void) { return interface_get(IF_UART); }

void bno086_begin(void) {
    Stream *c = console();

    /* --- Physical bring-up ONLY: reset + strapping --- *
     * Strapping pins (BOOTN, CLKSEL0; PS0/PS1 are board-fixed for UART-SHTP)
     * are sampled at the RISING edge of reset, so they must be driven BEFORE
     * releasing NRST.
     *
     * This function deliberately does NOT touch the IMU UART - not begin(), not
     * read(). The 7Semi library (imu_source.cpp) is the SOLE owner of the IMU
     * UART: it begin()s the port and performs every read. An earlier diagnostic
     * "boot peek" here begin()'d and drained the UART, which left the link in a
     * half-initialized, mid-frame state and made the library's first Set-Feature
     * exchange fail (reports never enabled). Removing it lets the library own a
     * clean stream from reset.
     *
     * NOTE: the library's UART bus is constructed with rstPin = -1 precisely so
     * it does NOT pulse NRST itself - this function is the single owner of the
     * reset + strapping sequence, which the library's plain reset pulse could
     * not do (it never drives BOOTN/CLKSEL0).
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

    /* Give the device a moment to come out of reset before the library begins
     * talking to it. Do NOT read the UART here (see note above). */
    delay(BNO_BOOT_SETTLE_MS);

    if (c) c->println("BNO086: reset + strapped (UART owned by 7Semi library)");
}

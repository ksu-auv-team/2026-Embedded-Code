#include "bno086.h"
#include "interfaces.h"
#include "config.h"

/* --- Control pins (this board) --- */
#define BNO_NRST_PIN    PB4   /* reset, active low            */
#define BNO_BOOTN_PIN   PA1   /* HIGH = normal (non-DFU)      */
#define BNO_HINTN_PIN   PB5   /* host interrupt, active LOW   */
#define BNO_CLKSEL0_PIN PA15  /* clock select: HIGH = external clock on CLK pin */

/* --- Timing (from CEVA sh2 reference HAL) --- */
#define BNO_RESET_LOW_MS     12     /* hold NRST low >=10 ms          */
#define BNO_READY_TIMEOUT_MS 2000   /* wait for H_INTN to assert low  */
#define BNO_RX_TIMEOUT_MS    1000   /* wait for the response frame    */

/* --- RFC1662 UART-SHTP framing --- */
#define RFC1662_FLAG    0x7E
#define RFC1662_ESCAPE  0x7D
#define PROTOCOL_SHTP   0x01

/* --- SHTP --- */
#define SHTP_CHANNEL_CONTROL          2
#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8

/* The debug console we print decoded info to. */
static Stream *console(void) { return interface_get(IF_UART); }
static Stream *imu(void)     { return interface_get(IF_IMU); }

/* Send one byte to the IMU, applying RFC1662 escaping. */
static void tx_escaped(Stream *s, uint8_t b) {
    if (b == RFC1662_FLAG || b == RFC1662_ESCAPE) {
        s->write(RFC1662_ESCAPE);
        s->write((uint8_t)(b ^ 0x20));
    } else {
        s->write(b);
    }
}

/* Send an SHTP packet (channel + payload) wrapped in a UART-SHTP frame. */
static void shtp_send(uint8_t channel, const uint8_t *payload, uint8_t len) {
    Stream *s = imu();
    if (!s) return;

    /* SHTP 4-byte header: length (incl. header) LSB/MSB, channel, sequence. */
    uint16_t total = (uint16_t)len + 4;
    uint8_t header[4] = {
        (uint8_t)(total & 0xFF),
        (uint8_t)(total >> 8),
        channel,
        0  /* sequence number */
    };

    s->write(RFC1662_FLAG);
    s->write(PROTOCOL_SHTP);
    for (int i = 0; i < 4; i++) tx_escaped(s, header[i]);
    for (int i = 0; i < len; i++) tx_escaped(s, payload[i]);
    s->write(RFC1662_FLAG);
}

/* Receive one UART-SHTP frame into buf (the SHTP packet, header+payload, with
 * the leading protocol-id byte stripped and escapes decoded). Returns the
 * number of bytes stored, or 0 on timeout/overflow. */
static size_t shtp_receive(uint8_t *buf, size_t maxlen, uint32_t timeout_ms) {
    Stream *s = imu();
    if (!s) return 0;

    uint32_t start = millis();
    bool in_frame = false;
    bool escaped = false;
    bool got_proto = false;
    size_t n = 0;

    while (millis() - start < timeout_ms) {
        if (!s->available()) continue;
        uint8_t b = (uint8_t)s->read();

        if (b == RFC1662_FLAG) {
            if (in_frame && n > 0) {
                return n;          /* closing flag -> frame complete */
            }
            /* opening flag (or empty open/close) -> (re)start */
            in_frame = true;
            escaped = false;
            got_proto = false;
            n = 0;
            continue;
        }
        if (!in_frame) continue;

        if (b == RFC1662_ESCAPE) { escaped = true; continue; }
        if (escaped) { b ^= 0x20; escaped = false; }

        if (!got_proto) {          /* first byte after flag is protocol id */
            got_proto = true;
            continue;
        }
        if (n < maxlen) buf[n++] = b;
        else { in_frame = false; }  /* overflow -> drop frame */
    }
    return 0;  /* timed out */
}

static void print_u32(Stream *c, const char *label, const uint8_t *p) {
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    c->print(label);
    c->println(v);
}

void bno086_begin(void) {
    Stream *c = console();

    /* --- Bring-up --- *
     * Strapping pins (BOOTN, CLKSEL0; PS0/PS1 are board-fixed) are sampled at
     * the RISING edge of reset, so they must be driven BEFORE releasing NRST.
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

    /* Wait for H_INTN to assert (go LOW) = device ready. Also watch for any
     * UART bytes arriving, so we can tell "device silent" from "INTN issue". */
    Stream *imu_s = imu();
    uint32_t start = millis();
    bool ready = false;
    bool saw_low = false;          /* H_INTN ever observed low */
    uint8_t hintn_at_start = digitalRead(BNO_HINTN_PIN);
    uint32_t rx_bytes = 0;
    while (millis() - start < BNO_READY_TIMEOUT_MS) {
        if (digitalRead(BNO_HINTN_PIN) == LOW) { saw_low = true; ready = true; break; }
        if (imu_s && imu_s->available()) { imu_s->read(); rx_bytes++; }
    }
    /* If INTN never dropped, keep counting any bytes for a moment longer. */
    if (!ready && imu_s) {
        uint32_t t = millis();
        while (millis() - t < 300) { if (imu_s->available()) { imu_s->read(); rx_bytes++; } }
    }
    if (c) {
        c->print("BNO086: reset. H_INTN idle="); c->print(hintn_at_start ? "HIGH" : "LOW");
        c->print(" ready="); c->print(ready ? "yes" : "TIMEOUT");
        c->print(" imu_rx_bytes="); c->println(rx_bytes);
    }
    (void)saw_low;

#if IMU_USART_MODE == IMU_MODE_SHTP
    /* The "get info" Product ID Request only exists in UART-SHTP mode; in
     * UART-RVC there is no command channel. */

    /* The device emits its SHTP advertisement on boot; drain it briefly so our
     * Product ID Response isn't mixed with the advertisement frame. */
    uint8_t scratch[256];
    shtp_receive(scratch, sizeof(scratch), 150);

    /* --- Send Product ID Request --- */
    uint8_t req[2] = { SHTP_REPORT_PRODUCT_ID_REQUEST, 0x00 };
    shtp_send(SHTP_CHANNEL_CONTROL, req, sizeof(req));

    /* --- Receive + decode the Product ID Response (report 0xF8) --- */
    uint8_t pkt[64];
    for (int attempt = 0; attempt < 4; attempt++) {
        size_t n = shtp_receive(pkt, sizeof(pkt), BNO_RX_TIMEOUT_MS);
        if (n < 4) continue;
        const uint8_t *d = pkt + 4;          /* skip SHTP header */
        size_t dlen = n - 4;
        if (dlen >= 14 && d[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            if (c) {
                c->println("BNO086 Product ID:");
                c->print("  reset cause: "); c->println(d[1]);
                c->print("  SW version:  ");
                c->print(d[2]); c->print('.'); c->print(d[3]); c->print('.');
                uint16_t patch = (uint16_t)d[12] | ((uint16_t)d[13] << 8);
                c->println(patch);
                print_u32(c, "  part number: ", d + 4);
                print_u32(c, "  build number: ", d + 8);
            }
            return;
        }
    }

    if (c) c->println("BNO086: no Product ID Response (check UART-SHTP mode/wiring)");
#else
    /* UART-RVC: no command channel. The device just streams; the router
     * forwards it. Nothing to query here. */
    (void)shtp_send; (void)shtp_receive; (void)print_u32;
    if (c) c->println("BNO086: UART-RVC mode (streaming; no Product ID query)");
#endif
}

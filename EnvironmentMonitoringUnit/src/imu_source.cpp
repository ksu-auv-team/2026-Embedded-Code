#include "imu_source.h"
#include "interfaces.h"
#include "imu_config.h"
#include <math.h>

/* Select the UART transport before pulling in the 7Semi driver: BnoSelect.h
 * uses this to pick BnoUARTBus as the bus implementation. We keep the library
 * ONLY for begin() + enableReport() (the SHTP TX / Set-Feature side, which
 * works). All RX deframing AND report parsing is done locally below, because
 * the library's BnoUARTBus::rx() can't deframe a continuous 3 Mbaud stream
 * (it bails on the first non-0x7E byte) and its processPacket() assumes a
 * fixed header offset our stream doesn't reliably match. */
#define BNO_USE_UART
#include <7Semi_BNO08x.h>

/* UART transport on the IMU HardwareSerial.
 *   rstPin  = -1: bno086_begin() owns reset + BOOTN/CLKSEL0 strapping.
 *   intnPin = -1: H_INTN (PB5) is not wired; do not gate on it.
 *   rxPin/txPin = -1: pins come from the HardwareSerial construction (STM32). */
static BnoUARTBus  s_bus(imu_hardware_serial(), IMU_BAUD_RATE,
                         /*rxPin*/ -1, /*txPin*/ -1,
                         /*intnPin*/ -1, /*rstPin*/ -1);
static BNO08x_7Semi s_imu(s_bus);

/* Report rate: 10 ms => 100 Hz, matching the old UART-RVC stream rate. */
static const uint32_t REPORT_INTERVAL_MS = 10;

static bool      s_has_packet;
static ImuPacket s_packet;

uint32_t g_rv_count = 0;   /* DIAGNOSTIC: parsed Rotation Vector reports */

/* --- SH-2 / SHTP report ids and fixed-point scales (from the BNO08x datasheet) */
#define SH2_TIMEBASE_REF    0xFB   /* Base Timestamp Reference report          */
#define SH2_LINEAR_ACCEL    0x04   /* Linear acceleration, m/s^2, Q8           */
#define SH2_ROTATION_VECTOR 0x05   /* Rotation vector quaternion, unitless, Q14 */
#define Q8_SCALE   256.0f
#define Q14_SCALE  16384.0f

static inline int16_t rd16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* --- Custom HDLC deframer + SHTP report parser ----------------------------- *
 * Read the IMU UART byte-by-byte, RESYNC on 0x7E (the resync the library's
 * rx() lacked), unescape 0x7D, strip the protocol-id byte, then parse the
 * resulting SHTP frame ourselves.
 *
 * SHTP-over-UART frame after deframing (protocol-id stripped):
 *   [0..1] length (LSB,MSB, includes the 4-byte header)
 *   [2]    channel (3 = INPUT for sensor reports)
 *   [3]    sequence
 *   [4..]  cargo: usually a Base Timestamp Reference (0xFB + 4 bytes) followed
 *          by one or more sensor reports: [id][seq][status][delay][data...]
 *
 * Rather than trust a single fixed offset (our stream occasionally shifts a
 * byte), we SCAN the cargo for the 0xFB timebase marker and parse the sensor
 * report that follows it; if no timebase is present we scan from the header
 * end. This tolerates the minor framing jitter seen on the wire. */

#define HDLC_FLAG 0x7E
#define HDLC_ESC  0x7D
#define SHTP_PID  0x01

static const size_t SHTP_FRAME_MAX = 128;

static uint8_t  s_frame[SHTP_FRAME_MAX];
static size_t   s_frame_len = 0;
static bool     s_in_frame  = false;
static bool     s_escaped   = false;
static bool     s_got_pid   = false;

/* Parse one sensor report starting at p (id,seq,status,delay,data...). Returns
 * the report length consumed, or 0 if the id isn't one we handle / truncated. */
static size_t parse_report(const uint8_t *p, size_t avail) {
    if (avail < 1) return 0;
    switch (p[0]) {
    case SH2_ROTATION_VECTOR: {           /* 4 + i,j,k,r (4*2) + accuracy(2) */
        if (avail < 14) return 0;
        float i = rd16(&p[4])  / Q14_SCALE;
        float j = rd16(&p[6])  / Q14_SCALE;
        float k = rd16(&p[8])  / Q14_SCALE;
        float r = rd16(&p[10]) / Q14_SCALE;
        /* quat -> ZYX Euler (radians) */
        float roll  = atan2f(2.0f * (r * i + j * k), 1.0f - 2.0f * (i * i + j * j));
        float sinp  = 2.0f * (r * j - k * i);
        sinp = sinp > 1.0f ? 1.0f : (sinp < -1.0f ? -1.0f : sinp);
        float pitch = asinf(sinp);
        float yaw   = atan2f(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
        const float C = 18000.0f / (float)M_PI;   /* rad -> centidegrees */
        s_packet.yaw   = (int16_t)lroundf(yaw   * C);
        s_packet.pitch = (int16_t)lroundf(pitch * C);
        s_packet.roll  = (int16_t)lroundf(roll  * C);
        s_packet.index++;
        s_has_packet = true;
        /* DIAGNOSTIC: confirm RV reports are being parsed (rate-limited). */
        extern uint32_t g_rv_count; g_rv_count++;
        return 14;
    }
    case SH2_LINEAR_ACCEL: {              /* 4 + x,y,z (3*2) */
        if (avail < 10) return 0;
        s_packet.accel_x = (int16_t)lroundf(rd16(&p[4]) / Q8_SCALE * 100.0f);
        s_packet.accel_y = (int16_t)lroundf(rd16(&p[6]) / Q8_SCALE * 100.0f);
        s_packet.accel_z = (int16_t)lroundf(rd16(&p[8]) / Q8_SCALE * 100.0f);
        return 10;
    }
    default:
        return 0;                         /* unknown id: caller advances by 1 */
    }
}

/* Parse a complete deframed SHTP frame. The exact position of the channel/seq
 * header bytes proved unreliable on this link (the deframed payload sometimes
 * starts a byte or two off), so we do NOT gate on the channel field. Instead we
 * SCAN the whole frame for the 0xFB Base Timestamp Reference that prefixes a
 * batch of input reports; the sensor report(s) follow it. If no timebase is
 * found, we fall back to scanning the whole frame for recognized report ids. */
static void parse_shtp_frame(const uint8_t *f, size_t n) {
    if (n < 6) return;

    /* Locate the timebase marker (0xFB). On this link the Base Timestamp Ref is
     * 4 bytes total (0xFB + 1 delta byte + 2 trailing bytes), so the first
     * sensor report's id is exactly 4 bytes after the 0xFB. (Empirically from
     * captured frames: "... FB dd 00 00 [report_id] [seq] [status] [delay] ..."). */
    size_t i = 0;
    bool found_tb = false;
    for (size_t k = 0; k + 4 < n; k++) {
        if (f[k] == SH2_TIMEBASE_REF) { i = k + 4; found_tb = true; break; }
    }
    if (!found_tb) i = 4;                 /* no timebase: start after SHTP header */

    /* Walk the cargo. Recognized reports advance by their length; anything else
     * advances by one byte (resync), so a stray/unknown id can't derail us.
     * Guard i <= n so the (n - i) length passed to parse_report never wraps. */
    while (i < n) {
        size_t used = parse_report(&f[i], n - i);
        i += (used ? used : 1);
    }
}

/* DIAGNOSTIC: dump the first N complete deframed frames in full, as hex, to the
 * VCP so the exact SHTP-over-UART byte layout can be analyzed offline. */
#define DUMP_FRAMES 200
static uint32_t s_dump_count = 0;
static void dump_frame(const uint8_t *f, size_t n) {
    if (s_dump_count >= DUMP_FRAMES) return;
    s_dump_count++;
    Stream *c = interface_get(IF_UART);
    if (!c) return;
    c->print("F["); c->print(n); c->print("]:");
    for (size_t i = 0; i < n; i++) {
        c->print(' ');
        if (f[i] < 16) c->print('0');
        c->print(f[i], HEX);
    }
    c->println();
}

static void deframe_byte(uint8_t b) {
    if (b == HDLC_FLAG) {
        if (s_in_frame && s_got_pid && s_frame_len >= 5) {
            dump_frame(s_frame, s_frame_len);
            parse_shtp_frame(s_frame, s_frame_len);
        }
        s_in_frame  = true;
        s_escaped   = false;
        s_got_pid   = false;
        s_frame_len = 0;
        return;
    }
    if (!s_in_frame) return;

    if (b == HDLC_ESC) { s_escaped = true; return; }
    if (s_escaped)     { b ^= 0x20; s_escaped = false; }

    if (!s_got_pid) {                     /* first post-flag byte is the protocol id */
        s_got_pid = true;
        if (b != SHTP_PID) s_in_frame = false;
        return;
    }

    /* A 0x7E closes the frame; per the SHTP length header we should keep
     * accumulating until the byte count matches. Some captured frames came up
     * short, suggesting an early flag or RX loss - track the expected length so
     * we can tell which. */
    if (s_frame_len < SHTP_FRAME_MAX) s_frame[s_frame_len++] = b;
    else                              s_in_frame = false;
}

void imu_source_setup(void) {
    s_has_packet = false;

    /* begin() initializes the UART (sole owner of IMU UART bring-up). */
    s_imu.begin();

    /* Enable the fused quaternion + gravity-free acceleration. The library's
     * Set-Feature WRITE works; we ignore its response ack (which is unreliable
     * at 3 Mbaud) and rely on the reports actually streaming. */
    s_imu.enableRotationVector(REPORT_INTERVAL_MS);
    s_imu.enableLinearAccel(REPORT_INTERVAL_MS);

    if (Stream *c = interface_get(IF_UART)) {
        c->println("IMU: reports enabled (RotationVector + LinearAccel)");
    }
}

void imu_source_update(void) {
    /* Drain whatever the IMU UART has buffered through our resync-capable
     * deframer; each complete frame is parsed inline (parse_shtp_frame), which
     * fills s_packet and sets s_has_packet. Cap the per-call byte count so a
     * flooded FIFO can't starve loop(); leftover bytes are read next tick. */
    HardwareSerial &imu = imu_hardware_serial();
    int budget = 512;
    while (imu.available() && budget-- > 0) {
        deframe_byte((uint8_t)imu.read());
    }

    /* DIAGNOSTIC: 1 Hz report of how many RV reports we've parsed. */
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms >= 1000) {
        last_ms = now;
        if (Stream *c = interface_get(IF_UART)) {
            c->print("IMU diag: rv_parsed="); c->println(g_rv_count);
        }
    }
}

bool imu_source_get(ImuPacket &out) {
    if (!s_has_packet) return false;
    out          = s_packet;
    s_has_packet = false;
    return true;
}

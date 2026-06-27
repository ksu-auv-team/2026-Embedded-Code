#include "imu_source.h"
#include "interfaces.h"
#include "imu_config.h"
#include <math.h>

/* Select the UART transport before pulling in the 7Semi driver: BnoSelect.h
 * uses this to pick BnoUARTBus as the bus implementation. */
#define BNO_USE_UART
#include <7Semi_BNO08x.h>

/* UART transport on the IMU HardwareSerial.
 *   rstPin  = -1: bno086_begin() owns reset + BOOTN/CLKSEL0 strapping.
 *   intnPin = -1: H_INTN (PB5) is NOT wired/functional on this board, so we do
 *                 NOT gate reads on it. BnoUARTBus::rx() bails out immediately
 *                 if it's told to wait for an INTN that never asserts; reading
 *                 the UART directly (its own 50 ms frame timeout handles "no
 *                 data yet") is correct here.
 *   rxPin/txPin = -1: pins come from the HardwareSerial construction (STM32). */
static BnoUARTBus  s_bus(imu_hardware_serial(), IMU_BAUD_RATE,
                         /*rxPin*/ -1, /*txPin*/ -1,
                         /*intnPin*/ -1, /*rstPin*/ -1);
static BNO08x_7Semi s_imu(s_bus);

/* Report rate: 10 ms => 100 Hz, matching the old UART-RVC stream rate. */
static const uint32_t REPORT_INTERVAL_MS = 10;

static bool      s_has_packet;
static ImuPacket s_packet;

/* DIAGNOSTIC counters (definitions; referenced via extern in deframe_byte). */
uint32_t g_frames_seen = 0;
uint8_t  g_last_ch     = 0xFF;
uint8_t  g_last_report = 0xFF;

/* --- Custom HDLC deframer -------------------------------------------------- *
 * The 7Semi library's BnoUARTBus::rx() bails out the moment the first byte it
 * reads isn't a 0x7E start flag. On a continuous 3 Mbaud SHTP stream the FIFO
 * is almost never sitting exactly on a frame boundary, so rx() succeeds only
 * "occasionally" and reports never decode reliably.
 *
 * We bypass rx() entirely: read the IMU UART byte-by-byte, RESYNC on 0x7E,
 * unescape 0x7D, strip the 1-byte protocol id, and hand the resulting SHTP
 * frame (header + payload) to the library's processPacket(), which still does
 * all the report parsing/caching. We keep using the library for begin(),
 * enableReport() (TX works), processPacket(), and the get*() accessors. */

#define HDLC_FLAG    0x7E
#define HDLC_ESC     0x7D
#define SHTP_PID     0x01

static const size_t SHTP_FRAME_MAX = 128;  /* RV + linaccel frames are < 64 */

static uint8_t  s_frame[SHTP_FRAME_MAX];
static size_t   s_frame_len  = 0;
static bool     s_in_frame   = false;
static bool     s_escaped    = false;
static bool     s_got_pid    = false;

/* Feed one raw UART byte through the deframer; on a complete frame, dispatch
 * it to the library's parser. */
static void deframe_byte(uint8_t b) {
    if (b == HDLC_FLAG) {
        /* A flag closes the current frame (if non-empty) and opens the next.
         * This is the resync point the library's rx() lacked. */
        if (s_in_frame && s_got_pid && s_frame_len > 0) {
            /* DIAGNOSTIC: tally frames and remember the last channel/report id
             * so we can see WHAT is arriving (control vs sensor reports). */
            extern uint32_t g_frames_seen;
            extern uint8_t  g_last_ch;
            extern uint8_t  g_last_report;
            g_frames_seen++;
            if (s_frame_len >= 10) {
                g_last_ch     = s_frame[2] & 0x0F;
                g_last_report = s_frame[9];
            }
            s_imu.processPacket(s_frame, s_frame_len);
        }
        s_in_frame  = true;
        s_escaped   = false;
        s_got_pid   = false;
        s_frame_len = 0;
        return;
    }
    if (!s_in_frame) return;            /* bytes before the first flag: ignore */

    if (b == HDLC_ESC) { s_escaped = true; return; }
    if (s_escaped)     { b ^= 0x20; s_escaped = false; }

    if (!s_got_pid) {                  /* first post-flag byte is the protocol id */
        s_got_pid = true;
        if (b != SHTP_PID) s_in_frame = false;  /* not SHTP: drop until next flag */
        return;
    }

    if (s_frame_len < SHTP_FRAME_MAX) {
        s_frame[s_frame_len++] = b;
    } else {
        s_in_frame = false;            /* overflow: drop, wait for next flag */
    }
}

void imu_source_setup(void) {
    s_has_packet = false;

    /* begin() initializes the UART (the single owner of IMU UART bring-up;
     * interfaces_begin() intentionally skips it). */
    s_imu.begin();

    /* Orientation as a fused quaternion, plus gravity-free acceleration -
     * together these reproduce the heading + accel fields of the old RVC frame. */
    bool rv  = s_imu.enableRotationVector(REPORT_INTERVAL_MS);
    bool lin = s_imu.enableLinearAccel(REPORT_INTERVAL_MS);

    /* DIAGNOSTIC: report whether Set-Feature was acknowledged. If these are
     * "no", the BNO never confirmed report enable (common at 3 Mbaud when the
     * control-channel response is missed) and no sensor frames will stream. */
    if (Stream *c = interface_get(IF_UART)) {
        c->print("IMU enable: rotationVector="); c->print(rv ? "OK" : "no");
        c->print(" linearAccel="); c->println(lin ? "OK" : "no");
    }
}

/* Convert a unit quaternion to ZYX Euler angles (yaw/pitch/roll) in radians.
 * SH-2 quaternion ordering is (i, j, k, r) with r the scalar component. */
static void quat_to_euler(float i, float j, float k, float r,
                          float &yaw, float &pitch, float &roll) {
    /* roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (r * i + j * k);
    float cosr_cosp = 1.0f - 2.0f * (i * i + j * j);
    roll = atan2f(sinr_cosp, cosr_cosp);

    /* pitch (y-axis rotation), clamped to avoid NaN at the poles */
    float sinp = 2.0f * (r * j - k * i);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = asinf(sinp);

    /* yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (r * k + i * j);
    float cosy_cosp = 1.0f - 2.0f * (j * j + k * k);
    yaw = atan2f(siny_cosp, cosy_cosp);
}

/* Scale radians to 1/100 degree (centidegrees), the ImuPacket fixed-point unit:
 * degrees = rad * 180/pi, centidegrees = degrees * 100 => rad * 18000/pi. */
static int16_t rad_to_centideg(float rad) {
    return (int16_t)lroundf(rad * (18000.0f / (float)M_PI));
}

void imu_source_update(void) {
    /* Drain whatever the IMU UART has buffered through our resync-capable
     * deframer. Cap the per-call byte count so a flooded FIFO can't starve the
     * rest of loop(); leftover bytes are picked up next iteration. */
    HardwareSerial &imu = imu_hardware_serial();
    int budget = 512;
    while (imu.available() && budget-- > 0) {
        deframe_byte((uint8_t)imu.read());
    }

    float qi, qj, qk, qr;
    if (s_imu.getQuaternion(qi, qj, qk, qr)) {
        float yaw, pitch, roll;
        quat_to_euler(qi, qj, qk, qr, yaw, pitch, roll);
        s_packet.yaw   = rad_to_centideg(yaw);
        s_packet.pitch = rad_to_centideg(pitch);
        s_packet.roll  = rad_to_centideg(roll);
        s_packet.index++;          /* rolling frame counter, as RVC provided */
        s_has_packet = true;
    }

    /* Fold in the latest linear acceleration whenever a fresh sample is ready;
     * scaled to 1/100 m/s^2 to match the ImuPacket fixed-point unit. */
    float ax, ay, az;
    if (s_imu.getLinearAccel(ax, ay, az)) {
        s_packet.accel_x = (int16_t)lroundf(ax * 100.0f);
        s_packet.accel_y = (int16_t)lroundf(ay * 100.0f);
        s_packet.accel_z = (int16_t)lroundf(az * 100.0f);
    }

    /* DIAGNOSTIC: once per second, report whether the library has decoded any
     * report at all (dataReady) since boot. Tells "no frames arriving" apart
     * from "frames arrive but quaternion not enabled". */
    static uint32_t last_diag_ms = 0;
    static bool ever_ready = false;
    if (s_imu.available()) ever_ready = true;
    uint32_t now = millis();
    if (now - last_diag_ms >= 1000) {
        last_diag_ms = now;
        if (Stream *c = interface_get(IF_UART)) {
            c->print("IMU diag: any_report_decoded="); c->println(ever_ready ? "yes" : "no");
        }
    }
}

bool imu_source_get(ImuPacket &out) {
    if (!s_has_packet) return false;
    out          = s_packet;
    s_has_packet = false;
    return true;
}

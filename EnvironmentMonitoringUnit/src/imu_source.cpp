#include "imu_source.h"
#include "interfaces.h"
#include "imu_config.h"
#include <math.h>

/* --- Circular DMA RX for the IMU UART (USART1) ----------------------------- *
 * At the BNO08x's fixed 3 Mbaud, interrupt-per-byte RX (what the Arduino core
 * uses) can't keep up and drops bytes mid-frame, corrupting HDLC sync. We let
 * the DMA controller copy every received byte into a circular RAM buffer with
 * zero CPU involvement, so no byte is ever lost. The core's HardwareSerial is
 * still used for begin() (pin/baud/clock) and TX (Set-Feature); we just take
 * over RX with our own DMA stream on the same USART1 instance.
 *
 * This is specific to IMU_SEL_USART1_PB6_PB7 / USART1; guarded accordingly. */
#if IMU_UART_SELECT == IMU_SEL_USART1_PB6_PB7 || IMU_UART_SELECT == IMU_SEL_USART1_PA9_PA10
  #define IMU_DMA_RX 1
  #include "stm32g4xx_ll_dma.h"
  #include "stm32g4xx_ll_usart.h"
  #include "stm32g4xx_ll_bus.h"

  #define IMU_DMA_BUF_SIZE 1024
  static volatile uint8_t s_dma_buf[IMU_DMA_BUF_SIZE];
  static size_t           s_dma_tail = 0;

  static void imu_dma_rx_begin(void) {
      /* USART1 was already configured (pins/baud) by HardwareSerial::begin().
       * Enable DMA1 clock, point DMA1 Channel1 at USART1->RDR in circular mode,
       * route the USART1_RX request (24) through DMAMUX, and turn on USART RX DMA. */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
      LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART1_RX);
      LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
          LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
          LL_DMA_MODE_CIRCULAR              |
          LL_DMA_PERIPH_NOINCREMENT         |
          LL_DMA_MEMORY_INCREMENT           |
          LL_DMA_PDATAALIGN_BYTE            |
          LL_DMA_MDATAALIGN_BYTE            |
          LL_DMA_PRIORITY_HIGH);
      LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
          LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
          (uint32_t)s_dma_buf,
          LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, IMU_DMA_BUF_SIZE);
      LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

      LL_USART_EnableDMAReq_RX(USART1);
      s_dma_tail = 0;
  }

  /* Current DMA write position (head) within the circular buffer. */
  static inline size_t imu_dma_head(void) {
      return IMU_DMA_BUF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
  }
#endif

/* This module talks to the BNO086 over UART-SHTP directly, with no external
 * driver dependency: it sends SHTP Set-Feature commands (TX), captures the
 * continuous 3 Mbaud stream via circular DMA, deframes the HDLC/SHTP envelope,
 * and parses the sensor reports - all self-contained below. */

/* Report rate: 10 ms => 100 Hz, matching the old UART-RVC stream rate. */
static const uint32_t REPORT_INTERVAL_MS = 10;

/* Bring-up timing: wait for the BNO to finish its post-reset advertisement
 * before commanding it, and the inter-command gap when enabling reports. */
static const uint32_t BOOT_SETTLE_MS       = 200;
static const uint32_t REPORT_ENABLE_GAP_MS = 20;

static bool      s_has_packet;
static ImuPacket s_packet;

/* --- SHTP / SH-2 protocol constants (BNO08x datasheet / SH-2 reference) --- */
#define SHTP_CH_CONTROL     2      /* control channel (Set-Feature / commands) */
#define SH2_CMD_SET_FEATURE 0xFD   /* Set Feature Command report id            */
#define SH2_CMD_REQUEST     0xF2   /* Command Request report id                */
#define SH2_TIMEBASE_REF    0xFB   /* Base Timestamp Reference report          */
#define SH2_LINEAR_ACCEL    0x04   /* Linear acceleration, m/s^2, Q8           */
#define SH2_ROTATION_VECTOR 0x05   /* Rotation vector quaternion, unitless, Q14 */

/* SH-2 commands carried in the Command Request (0xF2) report. */
#define SH2_COMMAND_SAVE_DCD       0x06  /* persist calibration to BNO flash    */
#define SH2_COMMAND_ME_CALIBRATE   0x07  /* configure dynamic (ME) calibration  */

/* SH-2 fixed-point scales: raw int16 / scale = engineering units. */
#define Q8_SCALE   256.0f          /* m/s^2 (accel)                            */
#define Q14_SCALE  16384.0f        /* unitless (quaternion)                    */

/* Sensor report layout: [id][seq][status][delay] header, then int16 LE data. */
#define REPORT_HEADER_BYTES 4
#define REPORT_LEN_RV   (REPORT_HEADER_BYTES + 4 * 2)   /* i,j,k,r            */
#define REPORT_LEN_ACC  (REPORT_HEADER_BYTES + 3 * 2)   /* x,y,z             */

/* HDLC framing for UART-SHTP: 0x7E flags, 0x7D escape (byte ^ 0x20). */
#define HDLC_FLAG 0x7E
#define HDLC_ESC  0x7D
#define SHTP_PID  0x01

/* ImuPacket units: angles in centidegrees, accel in 1/100 m/s^2. */
#define RAD_TO_CENTIDEG (18000.0f / (float)M_PI)
#define MPS2_TO_CENTI   100.0f

/* Read a signed 16-bit little-endian value from a byte pointer. */
static inline int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

/* --- SHTP transmit ---------------------------------------------------------- *
 * All control traffic (Set-Feature, Command Request) is built as an SHTP
 * payload on the control channel, then HDLC-wrapped over the IMU UART. We do
 * NOT wait for command responses (unreliable to catch at 3 Mbaud); commands
 * simply take effect, and the resulting reports stream in for our RX path. */
static uint8_t s_ctrl_seq = 0;     /* SHTP control-channel sequence number      */
static uint8_t s_cmd_seq  = 0;     /* SH-2 Command Request sequence number       */

/* Build the 4-byte SHTP header for an SHTP payload of total length 'len'
 * (header + cargo) on the control channel, then HDLC-frame and send it. The
 * caller fills f[4..] with the SH-2 report; f[0..3] are written here. */
static void shtp_send_frame(uint8_t *f, size_t len) {
    f[0] = len & 0xFF;                /* SHTP length LSB (incl. 4-byte header)   */
    f[1] = len >> 8;                  /* SHTP length MSB                         */
    f[2] = SHTP_CH_CONTROL;           /* channel                                 */
    f[3] = s_ctrl_seq++;              /* SHTP sequence                           */

    HardwareSerial &s = imu_hardware_serial();
    s.write(HDLC_FLAG);
    s.write(SHTP_PID);
    for (size_t i = 0; i < len; i++) {
        uint8_t b = f[i];
        if (b == HDLC_FLAG || b == HDLC_ESC) { s.write(HDLC_ESC); s.write((uint8_t)(b ^ 0x20)); }
        else                                 { s.write(b); }
    }
    s.write(HDLC_FLAG);
    s.flush();
}

/* Set Feature Command (0xFD): request periodic output of 'report_id'. */
static void shtp_tx_set_feature(uint8_t report_id, uint32_t interval_ms) {
    const uint32_t us = interval_ms * 1000UL;
    uint8_t f[21] = {0};
    f[4] = SH2_CMD_SET_FEATURE;
    f[5] = report_id;                 /* feature / report id                    */
    /* f[6..8] = flags + change sensitivity (0) */
    f[9]  =  us        & 0xFF;        /* report interval (us), LE               */
    f[10] = (us >> 8)  & 0xFF;
    f[11] = (us >> 16) & 0xFF;
    f[12] = (us >> 24) & 0xFF;
    /* f[13..20] = batch interval + reserved (0) */
    shtp_send_frame(f, sizeof(f));
}

/* Command Request (0xF2): issue SH-2 command 'cmd' with parameters p[0..8]. */
static void shtp_tx_command(uint8_t cmd, const uint8_t p[9]) {
    uint8_t f[16] = {0};              /* 4 SHTP hdr + 0xF2 + seq + cmd + 9 params */
    f[4] = SH2_CMD_REQUEST;
    f[5] = s_cmd_seq++;               /* command sequence number                */
    f[6] = cmd;
    for (int i = 0; i < 9; i++) f[7 + i] = p ? p[i] : 0;
    shtp_send_frame(f, sizeof(f));
}

/* Enable/disable dynamic (ME) calibration for accel, gyro and mag. */
static void imu_set_autocal(bool on) {
    const uint8_t e = on ? 1 : 0;
    /* P0=accel P1=gyro P2=mag P3=0(Configure ME Cal) P4=planar accel */
    const uint8_t p[9] = { e, e, e, 0, e, 0, 0, 0, 0 };
    shtp_tx_command(SH2_COMMAND_ME_CALIBRATE, p);
}

/* Persist the current dynamic calibration to the BNO086's flash (Save DCD). */
static void imu_save_calibration(void) {
    const uint8_t p[9] = {0};
    shtp_tx_command(SH2_COMMAND_SAVE_DCD, p);
}

/* Enable a periodic sensor report. The BNO ignores commands sent before it has
 * finished its post-reset advertisement, and may miss a single command, so we
 * send the Set-Feature twice with a short gap. We do not wait for / require the
 * command response (unreliable at 3 Mbaud); the report simply starts streaming. */
static void enable_report(uint8_t report_id, uint32_t interval_ms) {
    shtp_tx_set_feature(report_id, interval_ms);
    delay(REPORT_ENABLE_GAP_MS);
    shtp_tx_set_feature(report_id, interval_ms);
    delay(REPORT_ENABLE_GAP_MS);
}

/* --- HDLC deframer + SHTP report parser ------------------------------------ *
 * Process the IMU UART byte-by-byte: RESYNC on 0x7E, unescape 0x7D, strip the
 * protocol-id byte, then parse the resulting SHTP frame. Resyncing on every
 * flag (rather than bailing on the first non-flag byte) is what makes this work
 * on the BNO's continuous 3 Mbaud stream.
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

static const size_t SHTP_FRAME_MAX = 128;

static uint8_t  s_frame[SHTP_FRAME_MAX];
static size_t   s_frame_len = 0;
static bool     s_in_frame  = false;
static bool     s_escaped   = false;
static bool     s_got_pid   = false;

#if CAL_ENABLE_CONTROLLER
/* Dynamic-calibration controller with hysteresis (see imu_config.h section 5).
 * Driven by each Rotation Vector report's accuracy (0..3). Autocal starts OFF
 * (the device uses its saved calibration); we re-enable it when accuracy drops
 * too low, and Save-DCD + disable once it has recovered. Edge-triggered so we
 * issue a command only on a state change, not every frame. */
static bool s_autocal_on = false;

static void calibration_controller(uint8_t accuracy) {
    if (!s_autocal_on && accuracy < CAL_REENABLE_LEVEL) {
        imu_set_autocal(true);
        s_autocal_on = true;
    } else if (s_autocal_on && accuracy >= CAL_SAVE_LEVEL) {
        imu_save_calibration();
        imu_set_autocal(false);
        s_autocal_on = false;
    }
}
#endif

/* Convert a unit quaternion (SH-2 i,j,k,r order, r = scalar) to ZYX Euler
 * angles in radians. */
static void quat_to_euler(float i, float j, float k, float r,
                          float &yaw, float &pitch, float &roll) {
    roll = atan2f(2.0f * (r * i + j * k), 1.0f - 2.0f * (i * i + j * j));
    float sinp = 2.0f * (r * j - k * i);
    sinp = sinp > 1.0f ? 1.0f : (sinp < -1.0f ? -1.0f : sinp);  /* clamp at poles */
    pitch = asinf(sinp);
    yaw = atan2f(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
}

/* Parse one sensor report starting at p (id,seq,status,delay,data...). Returns
 * the report length consumed, or 0 if the id isn't one we handle / truncated.
 * Decoded values are stored into s_packet (centidegrees / centi-m/s^2). */
static size_t parse_report(const uint8_t *p, size_t avail) {
    if (avail < 1) return 0;
    const uint8_t *data = p + REPORT_HEADER_BYTES;
    switch (p[0]) {
    case SH2_ROTATION_VECTOR: {
        if (avail < REPORT_LEN_RV) return 0;
        float yaw, pitch, roll;
        quat_to_euler(read_i16_le(&data[0]) / Q14_SCALE,
                      read_i16_le(&data[2]) / Q14_SCALE,
                      read_i16_le(&data[4]) / Q14_SCALE,
                      read_i16_le(&data[6]) / Q14_SCALE,
                      yaw, pitch, roll);
        s_packet.yaw   = (int16_t)lroundf(yaw   * RAD_TO_CENTIDEG);
        s_packet.pitch = (int16_t)lroundf(pitch * RAD_TO_CENTIDEG);
        s_packet.roll  = (int16_t)lroundf(roll  * RAD_TO_CENTIDEG);
        /* status byte (report header [id][seq][status][delay]); low 2 bits =
         * fusion accuracy 0..3. */
        s_packet.accuracy = p[2] & 0x03;
        s_packet.index++;
        s_has_packet = true;
#if CAL_ENABLE_CONTROLLER
        calibration_controller(s_packet.accuracy);
#endif
        return REPORT_LEN_RV;
    }
    case SH2_LINEAR_ACCEL: {
        if (avail < REPORT_LEN_ACC) return 0;
        s_packet.accel_x = (int16_t)lroundf(read_i16_le(&data[0]) / Q8_SCALE * MPS2_TO_CENTI);
        s_packet.accel_y = (int16_t)lroundf(read_i16_le(&data[2]) / Q8_SCALE * MPS2_TO_CENTI);
        s_packet.accel_z = (int16_t)lroundf(read_i16_le(&data[4]) / Q8_SCALE * MPS2_TO_CENTI);
        return REPORT_LEN_ACC;
    }
    default:
        return 0;                         /* unknown id: caller advances by 1 */
    }
}

/* Parse a complete deframed SHTP frame. SHTP input reports are prefixed by a
 * 0xFB Base Timestamp Reference; rather than depend on a fixed header offset we
 * SCAN for that 0xFB and parse the sensor report(s) that follow it (with a
 * byte-by-byte resync fallback if no timebase is present). This is robust to
 * the channel/seq header bytes varying in position between frames. */
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

static void deframe_byte(uint8_t b) {
    if (b == HDLC_FLAG) {
        /* A flag closes the current frame and opens the next - resyncing on
         * every flag is what keeps us aligned on the continuous stream. */
        if (s_in_frame && s_got_pid && s_frame_len >= 5) {
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

    if (s_frame_len < SHTP_FRAME_MAX) s_frame[s_frame_len++] = b;
    else                              s_in_frame = false;   /* overflow: resync at next flag */
}

void imu_source_setup(void) {
    s_has_packet = false;

    /* Bring up the IMU UART (pins/baud/clock). This module is the sole owner of
     * the IMU UART; interfaces_begin() intentionally does not begin() it. */
    imu_hardware_serial().begin(IMU_BAUD_RATE);

#if IMU_DMA_RX
    /* Take over RX with circular DMA so no bytes drop at 3 Mbaud. Must run after
     * the UART begin() above has configured USART1 pins/baud/clock. */
    imu_dma_rx_begin();
#endif

    /* Wait for the chip to finish booting, then enable the fused quaternion
     * (orientation) and gravity-free acceleration reports. */
    delay(BOOT_SETTLE_MS);
    enable_report(SH2_ROTATION_VECTOR, REPORT_INTERVAL_MS);
    enable_report(SH2_LINEAR_ACCEL,    REPORT_INTERVAL_MS);

    if (Stream *c = interface_get(IF_UART)) {
        c->println("IMU: reports enabled (RotationVector + LinearAccel)");
    }
}

void imu_source_update(void) {
#if IMU_DMA_RX
    /* Consume every byte the DMA has written since last call, in order, through
     * the deframer. The DMA captures the full 3 Mbaud stream with no CPU
     * involvement, so no byte is ever dropped and frames arrive intact. Each
     * complete frame is parsed inline (parse_shtp_frame), filling s_packet. */
    size_t head = imu_dma_head();
    while (s_dma_tail != head) {
        deframe_byte(s_dma_buf[s_dma_tail]);
        s_dma_tail = (s_dma_tail + 1) & (IMU_DMA_BUF_SIZE - 1);
        head = imu_dma_head();   /* re-read: more may have arrived while parsing */
    }
#else
    /* Fallback (non-USART1 builds): interrupt-driven RX. Note this drops bytes
     * at 3 Mbaud - DMA RX above is required for reliable SHTP framing. */
    HardwareSerial &imu = imu_hardware_serial();
    int budget = 512;
    while (imu.available() && budget-- > 0) {
        deframe_byte((uint8_t)imu.read());
    }
#endif
}

bool imu_source_get(ImuPacket &out) {
    if (!s_has_packet) return false;
    out          = s_packet;
    s_has_packet = false;
    return true;
}

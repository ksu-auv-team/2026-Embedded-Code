#include "imu_reader.h"
#include "interfaces.h"

/* BNO086 UART-RVC frame (19 bytes total):
 *
 *  [0]   0xAA  header
 *  [1]   0xAA  header
 *  [2]   index         rolling counter 0-255
 *  [3-4] yaw           int16 LE, 1/100 deg
 *  [5-6] pitch         int16 LE, 1/100 deg
 *  [7-8] roll          int16 LE, 1/100 deg
 *  [9-10] accel_x      int16 LE, 1/100 m/s^2
 *  [11-12] accel_y     int16 LE, 1/100 m/s^2
 *  [13-14] accel_z     int16 LE, 1/100 m/s^2
 *  [15]  motion accuracy  0=unreliable .. 3=high
 *  [16]  reserved
 *  [17]  reserved
 *  [18]  checksum      (sum of bytes [2..17]) & 0xFF
 *
 * COLLECT_LEN = 17: bytes [2..18] accumulated before parse/validate.
 */

static const int COLLECT_LEN = 17;

enum RvcState { WAIT_AA1, WAIT_AA2, COLLECT };

static RvcState  s_state;
static uint8_t   s_buf[COLLECT_LEN];
static int       s_pos;

static bool      s_has_packet;
static ImuPacket s_packet;

void imu_reader_setup(void) {
    s_state      = WAIT_AA1;
    s_pos        = 0;
    s_has_packet = false;
}

static void parse_frame(void) {
    /* Validate checksum: sum of buf[0..15] == buf[16] */
    uint8_t sum = 0;
    for (int i = 0; i < 16; i++) sum += s_buf[i];
    if (sum != s_buf[16]) return;

    s_packet.index   = s_buf[0];
    s_packet.yaw     = (int16_t)(s_buf[1]  | (s_buf[2]  << 8));
    s_packet.pitch   = (int16_t)(s_buf[3]  | (s_buf[4]  << 8));
    s_packet.roll    = (int16_t)(s_buf[5]  | (s_buf[6]  << 8));
    s_packet.accel_x = (int16_t)(s_buf[7]  | (s_buf[8]  << 8));
    s_packet.accel_y = (int16_t)(s_buf[9]  | (s_buf[10] << 8));
    s_packet.accel_z = (int16_t)(s_buf[11] | (s_buf[12] << 8));
    s_has_packet = true;
}

void imu_reader_update(void) {
    Stream *imu = interface_get(IF_IMU);
    if (!imu) return;

    while (imu->available()) {
        uint8_t b = (uint8_t)imu->read();

        switch (s_state) {
        case WAIT_AA1:
            if (b == 0xAA) s_state = WAIT_AA2;
            break;

        case WAIT_AA2:
            if (b == 0xAA) {
                s_state = COLLECT;
                s_pos   = 0;
            } else {
                /* Not a second 0xAA — check if it restarts a header */
                s_state = (b == 0xAA) ? WAIT_AA2 : WAIT_AA1;
            }
            break;

        case COLLECT:
            s_buf[s_pos++] = b;
            if (s_pos == COLLECT_LEN) {
                parse_frame();
                s_state = WAIT_AA1;
            }
            break;
        }
    }
}

bool imu_reader_get(ImuPacket &out) {
    if (!s_has_packet) return false;
    out          = s_packet;
    s_has_packet = false;
    return true;
}

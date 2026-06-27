#ifndef IMU_PACKET_H
#define IMU_PACKET_H

#include <stdint.h>

/* Parsed output of one BNO086 UART-RVC frame.
 *
 * Raw units: angles in 1/100 degrees, accelerations in 1/100 m/s^2.
 * Divide by 100.0f to get degrees / m/s^2.
 *
 * Sent over I2C as a packed binary struct (14 bytes).
 */
struct ImuPacket {
    uint8_t index;     // BNO086 rolling frame counter (0-255)
    int16_t yaw;       // 1/100 deg,   range -18000..18000
    int16_t pitch;     // 1/100 deg,   range  -9000..9000
    int16_t roll;      // 1/100 deg,   range  -9000..9000
    int16_t accel_x;   // 1/100 m/s^2
    int16_t accel_y;
    int16_t accel_z;
    uint8_t accuracy;  // BNO fusion accuracy/status: 0=unreliable .. 3=high
} __attribute__((packed));

#endif // IMU_PACKET_H

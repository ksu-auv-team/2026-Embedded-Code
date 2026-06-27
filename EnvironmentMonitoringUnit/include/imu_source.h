#ifndef IMU_SOURCE_H
#define IMU_SOURCE_H

#include "imu_packet.h"
#include <stdbool.h>

/* Self-contained BNO086 driver over UART-SHTP (no external library).
 *
 * Replaces the old hand-rolled UART-RVC parser (imu_reader). This module:
 *   - brings up the IMU UART with circular DMA RX (no byte loss at 3 Mbaud),
 *   - sends SHTP Set-Feature to enable the Rotation Vector (orientation) and
 *     Linear Acceleration reports,
 *   - deframes the HDLC/SHTP stream and parses those reports, converting each
 *     rotation-vector quaternion to yaw/pitch/roll and packing it, together with
 *     the latest linear acceleration, into an ImuPacket.
 *
 * Call imu_source_setup() once from setup() (after bno086_begin() has reset and
 * strapped the chip), then imu_source_update() every loop iteration.
 * imu_source_get() returns true (and fills 'out') once per new orientation
 * frame; false if no new frame is ready.
 */

void imu_source_setup(void);
void imu_source_update(void);
bool imu_source_get(ImuPacket &out);

#endif // IMU_SOURCE_H

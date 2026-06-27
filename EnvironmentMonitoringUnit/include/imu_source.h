#ifndef IMU_SOURCE_H
#define IMU_SOURCE_H

#include "imu_packet.h"
#include <stdbool.h>

/* BNO086 driver built on the 7Semi BNO08x library (UART-SHTP transport).
 *
 * Replaces the old hand-rolled UART-RVC parser (imu_reader). The library owns
 * SHTP framing and report decoding; this module:
 *   - constructs the UART bus on the IMU HardwareSerial,
 *   - enables the Rotation Vector (orientation) and Linear Acceleration reports,
 *   - converts each rotation-vector quaternion to yaw/pitch/roll and packs it,
 *     together with the latest linear acceleration, into an ImuPacket.
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

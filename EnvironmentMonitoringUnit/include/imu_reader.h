#ifndef IMU_READER_H
#define IMU_READER_H

#include "imu_packet.h"
#include <stdbool.h>

/* Parse the BNO086 UART-RVC byte stream into ImuPacket frames.
 *
 * Call imu_reader_setup() once from setup(), then imu_reader_update() every
 * loop iteration. imu_reader_get() returns true (and fills 'out') exactly once
 * per new validated frame; returns false if no new frame is ready.
 */

void imu_reader_setup(void);
void imu_reader_update(void);
bool imu_reader_get(ImuPacket &out);

#endif // IMU_READER_H

#ifndef DATA_PUBLISHER_H
#define DATA_PUBLISHER_H

#include "imu_packet.h"

/* Publish a parsed IMU frame.
 *
 *   - I2C slave: the STM32 responds at I2C_SLAVE_ADDR; an external host (the
 *     bus master) reads the latest ImuPacket on demand. Each published frame
 *     updates the snapshot the slave serves (config.h section 7).
 *   - Debug VCP (IF_UART): human-readable CSV line, throttled to
 *     PUBLISH_RATE_HZ (native USB is unused on this board).
 *
 * Call data_publisher_setup() once from setup(), then pass each new frame
 * to data_publisher_publish().
 */

void data_publisher_setup(void);
void data_publisher_publish(const ImuPacket &pkt);

#endif // DATA_PUBLISHER_H

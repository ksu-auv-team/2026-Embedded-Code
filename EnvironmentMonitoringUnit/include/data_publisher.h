#ifndef DATA_PUBLISHER_H
#define DATA_PUBLISHER_H

#include "imu_packet.h"

/* Publish a parsed IMU frame to configured outputs.
 *
 *   - Debug VCP (IF_UART): human-readable CSV line (native USB is unused on
 *     this board).
 *   - I2C output bus: raw ImuPacket struct, always sent to I2C_OUT_ADDR
 *     (configured in config.h section 7).
 *
 * Call data_publisher_setup() once from setup(), then pass each new frame
 * to data_publisher_publish().
 */

void data_publisher_setup(void);
void data_publisher_publish(const ImuPacket &pkt);

#endif // DATA_PUBLISHER_H

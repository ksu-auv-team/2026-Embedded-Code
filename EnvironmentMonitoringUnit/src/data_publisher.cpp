#include "data_publisher.h"
#include "config.h"
#include "interfaces.h"
#include <Wire.h>

/* I2C master instance on the output bus (SDA/SCL from config.h section 7). */
static TwoWire i2c_out(I2C_OUT_SDA, I2C_OUT_SCL);

void data_publisher_setup(void) {
    i2c_out.begin();
    i2c_out.setClock(I2C_OUT_SPEED);
}

void data_publisher_publish(const ImuPacket &pkt) {
    /* Throttle to PUBLISH_RATE_HZ: the IMU streams at ~100 Hz, but publishing
     * every frame floods the 115200 VCP. Drop frames between publish ticks.
     * PUBLISH_RATE_HZ == 0 disables the throttle (publish every frame). */
#if PUBLISH_RATE_HZ > 0
    static uint32_t last_publish_ms = 0;
    const uint32_t period_ms = 1000UL / PUBLISH_RATE_HZ;
    uint32_t now = millis();
    if (now - last_publish_ms < period_ms) return;
    last_publish_ms = now;
#endif

    /* --- Debug VCP: human-readable line --------------------------------- *
     * Native USB on this board is non-functional, so the human-readable frame
     * goes to the ST-Link VCP (IF_UART) instead of USB. */
    Stream *dbg = interface_get(IF_UART);
    if (dbg) {
        /* Format: YAW:ddd.dd,PITCH:ddd.dd,ROLL:ddd.dd,AX:d.dd,AY:d.dd,AZ:d.dd */
        dbg->print("YAW:");    dbg->print(pkt.yaw   / 100.0f, 2);
        dbg->print(",PITCH:"); dbg->print(pkt.pitch / 100.0f, 2);
        dbg->print(",ROLL:");  dbg->print(pkt.roll  / 100.0f, 2);
        dbg->print(",AX:");    dbg->print(pkt.accel_x / 100.0f, 2);
        dbg->print(",AY:");    dbg->print(pkt.accel_y / 100.0f, 2);
        dbg->print(",AZ:");    dbg->println(pkt.accel_z / 100.0f, 2);
    }

    /* --- I2C: binary ImuPacket struct, always ----------------------------- */
    i2c_out.beginTransmission(I2C_OUT_ADDR);
    i2c_out.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
    i2c_out.endTransmission();
}

#include "data_publisher.h"
#include "config.h"
#include <Wire.h>

/* I2C master instance on the output bus (SDA/SCL from config.h section 7). */
static TwoWire i2c_out(I2C_OUT_SDA, I2C_OUT_SCL);

void data_publisher_setup(void) {
    i2c_out.begin();
    i2c_out.setClock(I2C_OUT_SPEED);
}

void data_publisher_publish(const ImuPacket &pkt) {
    /* --- USB: human-readable line, only when host is connected ------------ */
#if USB_AVAILABLE
    if ((bool)USB_SERIAL) {
        /* Format: YAW:ddd.dd,PITCH:ddd.dd,ROLL:ddd.dd,AX:d.dd,AY:d.dd,AZ:d.dd */
        USB_SERIAL.print("YAW:");   USB_SERIAL.print(pkt.yaw   / 100.0f, 2);
        USB_SERIAL.print(",PITCH:"); USB_SERIAL.print(pkt.pitch / 100.0f, 2);
        USB_SERIAL.print(",ROLL:"); USB_SERIAL.print(pkt.roll  / 100.0f, 2);
        USB_SERIAL.print(",AX:");   USB_SERIAL.print(pkt.accel_x / 100.0f, 2);
        USB_SERIAL.print(",AY:");   USB_SERIAL.print(pkt.accel_y / 100.0f, 2);
        USB_SERIAL.print(",AZ:");   USB_SERIAL.println(pkt.accel_z / 100.0f, 2);
    }
#endif

    /* --- I2C: binary ImuPacket struct, always ----------------------------- */
    i2c_out.beginTransmission(I2C_OUT_ADDR);
    i2c_out.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
    i2c_out.endTransmission();
}

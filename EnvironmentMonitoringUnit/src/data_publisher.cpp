#include "data_publisher.h"
#include "config.h"
#include "interfaces.h"
#include <Wire.h>

/* I2C SLAVE: the STM32 responds at I2C_SLAVE_ADDR; an external host (master)
 * reads the latest ImuPacket on a plain I2C read. See config.h section 7. */
static TwoWire i2c_slave(I2C_SLAVE_SDA, I2C_SLAVE_SCL);

/* Latest packet, double-buffered. The main loop fills the inactive buffer then
 * flips s_active in a single (atomic) write; the onRequest ISR reads s_active
 * once and serves that buffer. This hands the ISR a fully consistent 14-byte
 * snapshot without disabling interrupts or risking a torn struct. */
static ImuPacket        s_buf[2];
static volatile uint8_t s_active = 0;

/* I2C read from the host: serve the most recently published packet. Runs in
 * ISR context - keep it to a single buffered write, no blocking work. */
static void on_request(void) {
    uint8_t idx = s_active;     /* read once: producer may flip concurrently */
    i2c_slave.write(reinterpret_cast<const uint8_t *>(&s_buf[idx]), sizeof(s_buf[idx]));
}

void data_publisher_setup(void) {
    s_buf[0] = ImuPacket{};     /* serve zeros until the first frame arrives */
    s_buf[1] = ImuPacket{};
    s_active = 0;

    i2c_slave.begin(I2C_SLAVE_ADDR);   /* slave mode: respond to our address */
    i2c_slave.onRequest(on_request);
}

void data_publisher_publish(const ImuPacket &pkt) {
    /* Update the I2C snapshot every frame so host reads always get the freshest
     * data: fill the inactive buffer, then flip the active index (one aligned
     * 32-bit write -> atomic w.r.t. the ISR). */
    uint8_t inactive = s_active ^ 1u;
    s_buf[inactive] = pkt;
    s_active = inactive;

    /* --- Debug VCP: human-readable line --------------------------------- *
     * Native USB on this board is non-functional, so the human-readable frame
     * goes to the ST-Link VCP (IF_UART). Throttled to PUBLISH_RATE_HZ so the
     * ~100 Hz stream doesn't flood the 115200 link (0 = print every frame).
     * The I2C snapshot above is NOT throttled - it updates every frame. */
#if PUBLISH_RATE_HZ > 0
    static uint32_t last_print_ms = 0;
    const uint32_t period_ms = 1000UL / PUBLISH_RATE_HZ;
    uint32_t now = millis();
    if (now - last_print_ms < period_ms) return;
    last_print_ms = now;
#endif

    Stream *dbg = interface_get(IF_UART);
    if (dbg) {
        /* Format: YAW:d.dd,PITCH:d.dd,ROLL:d.dd,AX:d.dd,AY:d.dd,AZ:d.dd,ACC:n
         * ACC is the BNO fusion accuracy 0 (unreliable) .. 3 (high). */
        dbg->print("YAW:");    dbg->print(pkt.yaw   / 100.0f, 2);
        dbg->print(",PITCH:"); dbg->print(pkt.pitch / 100.0f, 2);
        dbg->print(",ROLL:");  dbg->print(pkt.roll  / 100.0f, 2);
        dbg->print(",AX:");    dbg->print(pkt.accel_x / 100.0f, 2);
        dbg->print(",AY:");    dbg->print(pkt.accel_y / 100.0f, 2);
        dbg->print(",AZ:");    dbg->print(pkt.accel_z / 100.0f, 2);
        dbg->print(",ACC:");   dbg->println(pkt.accuracy);
    }
}

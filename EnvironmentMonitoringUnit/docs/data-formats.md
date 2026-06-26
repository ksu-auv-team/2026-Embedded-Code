# Data Formats

## BNO086 UART-RVC Frame (input)

In UART-RVC mode the BNO086 transmits 19-byte frames at 100 Hz with no framing handshake. The firmware parses these in `src/imu_reader.cpp`.

```
Byte  Field            Type      Scale             Notes
────  ──────────────── ───────── ───────────────── ──────────────────────────────
  0   Header 1         uint8     —                 Always 0xAA
  1   Header 2         uint8     —                 Always 0xAA
  2   Index            uint8     —                 Rolling counter 0–255
  3   Yaw LSB          }
  4   Yaw MSB          } int16 LE  1/100 degree    ÷100 → degrees (-180.00..180.00)
  5   Pitch LSB        }
  6   Pitch MSB        } int16 LE  1/100 degree    ÷100 → degrees (-90.00..90.00)
  7   Roll LSB         }
  8   Roll MSB         } int16 LE  1/100 degree    ÷100 → degrees (-90.00..90.00)
  9   Accel X LSB      }
 10   Accel X MSB      } int16 LE  1/100 m/s²      ÷100 → m/s²
 11   Accel Y LSB      }
 12   Accel Y MSB      } int16 LE  1/100 m/s²      ÷100 → m/s²
 13   Accel Z LSB      }
 14   Accel Z MSB      } int16 LE  1/100 m/s²      ÷100 → m/s²
 15   Motion accuracy  uint8     —                 0=unreliable, 1=low, 2=med, 3=high
 16   Reserved         uint8     —                 0x00
 17   Reserved         uint8     —                 0x00
 18   Checksum         uint8     —                 (sum of bytes [2..17]) & 0xFF
```

### Checksum

```c
uint8_t sum = 0;
for (int i = 2; i <= 17; i++) sum += frame[i];
valid = (sum == frame[18]);
```

Frames that fail the checksum are silently discarded by `imu_reader`.

### Parser state machine (`src/imu_reader.cpp`)

```
WAIT_AA1 ──(0xAA)──► WAIT_AA2 ──(0xAA)──► COLLECT 17 bytes
    ▲                    │                       │
    └────────────────────┘              checksum OK → ImuPacket ready
         (other byte)                            │
                                            → WAIT_AA1
```

---

## `ImuPacket` Struct (parsed output)

Defined in `include/imu_packet.h`. Packed with no padding (13 bytes total).

```c
struct ImuPacket {        // offset  size
    uint8_t index;        //  0       1 byte   rolling frame counter (0–255)
    int16_t yaw;          //  1       2 bytes  1/100 degree
    int16_t pitch;        //  3       2 bytes  1/100 degree
    int16_t roll;         //  5       2 bytes  1/100 degree
    int16_t accel_x;      //  7       2 bytes  1/100 m/s²
    int16_t accel_y;      //  9       2 bytes  1/100 m/s²
    int16_t accel_z;      // 11       2 bytes  1/100 m/s²
} __attribute__((packed));// total   13 bytes
```

All `int16_t` fields are little-endian (native ARM byte order).  
Divide by `100.0f` to convert to SI units (degrees or m/s²).

---

## USB Serial Monitor Output

Sent to `SerialUSB` only when a USB host is connected. Format: one CSV line per frame, `\r\n` terminated.

```
YAW:ddd.dd,PITCH:ddd.dd,ROLL:ddd.dd,AX:d.dd,AY:d.dd,AZ:d.dd
```

**Example** (115200 baud, PlatformIO serial monitor):

```
YAW:23.54,PITCH:-1.20,ROLL:0.87,AX:0.03,AY:-0.01,AZ:9.81
YAW:23.55,PITCH:-1.19,ROLL:0.88,AX:0.02,AY:-0.01,AZ:9.81
```

Values are floating-point strings with 2 decimal places, produced by Arduino's `Stream::print(float, 2)`. Rate mirrors the BNO086 output rate (~100 Hz in RVC mode).

---

## I2C Output Packet

The raw `ImuPacket` struct (13 bytes) is written to the slave at `I2C_OUT_ADDR` on every parsed frame (~100 Hz). No additional framing is added.

```
I2C write to I2C_OUT_ADDR (default 0x42):

 Byte   Field      Type      Value
 ─────  ─────────  ────────  ──────────────────────────────
  0     index      uint8     rolling counter 0–255
  1–2   yaw        int16 LE  angle × 100  (e.g. 2354 = 23.54°)
  3–4   pitch      int16 LE  angle × 100
  5–6   roll       int16 LE  angle × 100
  7–8   accel_x    int16 LE  accel × 100  (e.g. 981 = 9.81 m/s²)
  9–10  accel_y    int16 LE  accel × 100
 11–12  accel_z    int16 LE  accel × 100
```

### Receiving on a downstream device — Python (smbus2)

```python
import smbus2, struct

I2C_ADDR   = 0x42
BUS        = smbus2.SMBus(1)      # /dev/i2c-1 on Raspberry Pi

# '<Bhhhhhh' = 1 uint8 + 6 int16 little-endian = 13 bytes
PACKET_FMT = '<Bhhhhhh'

while True:
    raw = BUS.read_i2c_block_data(I2C_ADDR, 0, 13)
    idx, yaw, pitch, roll, ax, ay, az = struct.unpack(PACKET_FMT, bytes(raw))
    print(
        f"idx={idx:3d}  "
        f"yaw={yaw/100:7.2f}°  pitch={pitch/100:6.2f}°  roll={roll/100:6.2f}°  "
        f"ax={ax/100:6.2f}  ay={ay/100:6.2f}  az={az/100:6.2f} m/s²"
    )
```

### Receiving on a downstream device — C (e.g. Arduino / another STM32)

```c
#include <Wire.h>

#define EMU_ADDR  0x42

#pragma pack(push, 1)
typedef struct {
    uint8_t index;
    int16_t yaw, pitch, roll;
    int16_t accel_x, accel_y, accel_z;
} ImuPacket;
#pragma pack(pop)

void loop() {
    Wire.requestFrom(EMU_ADDR, sizeof(ImuPacket));
    if (Wire.available() == sizeof(ImuPacket)) {
        ImuPacket pkt;
        Wire.readBytes((uint8_t *)&pkt, sizeof(pkt));
        float yaw_deg = pkt.yaw / 100.0f;
        // ...
    }
}
```

> **Master vs slave:** The STM32 acts as **I2C master** and pushes data at 100 Hz. Standard Linux I2C drivers on a Raspberry Pi are master-only, so receiving requires either a secondary MCU acting as slave, a userspace slave driver (`i2c-slave-mqueue`), or swapping the roles in `data_publisher.cpp` to make the STM32 a slave that responds to polls.

# 2026-Embedded-Code
Collection of our Embedded Code

# I2C Addresses

| # | Address | Device Name | Data |
|-------|:---:|------------|------|
| 1 | 4a | Power Safety |
| 2 | 4b | IMU |
| 3 | 4c | ESC Controller |
| 4 | 4d | Display Controller |
| 5 | 4e | Hydrophone Controller |
| 6 | 4f | Torpedo Controller |
| 7 | 50 | Arm Controller |

# IMU I2C Packet

| Byte | Field | Type | Encoding |
|-------|:---:|------|------------|
| 0 | index | u8 | rolling 0–255 frame counter |
| 1-2 | yaw | i16 LE | 1/100° (−18000..18000) |
| 3-4 | pitch | i16 LE | 1/100° |
| 5-6 | roll | i16 LE | 1/100° |
| 7-8 | accel_x | i16 LE | 1/100 m/s² |
| 9-10 | accel_y | i16 LE | 1/100 m/s² |
| 11-12 | accel_z | i16 LE | 1/100 m/s² |
| 13 | accuracy | u8 | 0=unreliable .. 3-high |

The IMU I2C register is updated every 10ms (100Hz)

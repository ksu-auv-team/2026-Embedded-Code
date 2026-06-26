# Getting Started

## Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- ST-Link V2 programmer connected to the board's SWD pins
- BNO086 IMU wired as described in [hardware.md](hardware.md)

---

## 1. Wire the hardware

Minimum connections to get data flowing:

```
ST-Link ──SWD──► MCU (PA13 SWDIO, PA14 SWCLK, GND, 3V3)
BNO086  ──UART──► MCU (PB6 TX→IMU RX, PB7 RX←IMU TX, GND)
USB connector ──► MCU (PA11 D−, PA12 D+)  [for serial monitor]
```

For the I2C output bus, connect your downstream device to PA8 (SDA) and PA9 (SCL) with 2.2 kΩ pull-ups to 3.3 V.

Full pin reference: [hardware.md](hardware.md).

---

## 2. Configure

Open `include/config.h` and `include/imu_config.h`. Defaults that must match your board:

| Setting | Default | Check |
|---------|---------|-------|
| `IMU_USART_MODE` | `IMU_MODE_RVC` | Must match PS0/PS1 straps on BNO086 |
| `IMU_CLOCK` | `IMU_CLOCK_CRYSTAL` | Must match CLKSEL0 strap / board design |
| `I2C_OUT_ADDR` | `0x42` | Must match your downstream slave's address |
| `BAUD_RATE` | `115200` | Must match your UART terminal |

All other settings have sensible defaults. See [configuration.md](configuration.md) for a full reference.

---

## 3. Build and flash

```bash
pio run -t upload
```

Or use the **Upload** button in the VS Code PlatformIO toolbar.

The ST-Link programs via SWD at boot. No bootloader is required.

---

## 4. Verify with the serial monitor

**USB (PlatformIO monitor):**

```bash
pio device monitor
```

The monitor opens at 115200 baud (set by `monitor_speed` in `platformio.ini`). You should see IMU data lines at ~100 Hz:

```
YAW:0.00,PITCH:0.00,ROLL:0.00,AX:0.03,AY:-0.01,AZ:9.81
YAW:0.12,PITCH:-0.01,ROLL:0.00,AX:0.02,AY:-0.01,AZ:9.81
...
```

The heartbeat counter also appears once per second on the USB port (configurable via `HEARTBEAT_DESTINATIONS`).

**UART (ST-Link VCP):**

Open the ST-Link virtual COM port at 115200 baud in any terminal. At boot you will see:

```
BNO086: reset. H_INTN idle=HIGH ready=yes imu_rx_bytes=0
BNO086: UART-RVC mode (streaming; no Product ID query)
Environment Monitoring Unit Started
Heartbeat every 1000 ms, counter 0-255
```

---

## 5. Verify I2C output

With a logic analyser or oscilloscope on PA8/PA9 you should see 13-byte write transactions addressed to `I2C_OUT_ADDR` at ~100 Hz.

For a quick software check, connect a Raspberry Pi as the slave (requires a userspace I2C slave driver) or connect another Arduino as slave at address `0x42` and print the received bytes.

See [data-formats.md](data-formats.md) for the packet layout and ready-to-use Python/C receiver examples.

---

## Troubleshooting

**No USB output**
- Confirm the board enumerated (`dmesg | grep ttyACM` on Linux, Device Manager on Windows).
- PA11 must be D−, PA12 must be D+. Check the USB connector wiring.
- The firmware waits up to 2 s for the host at boot. If you miss the banner, press reset after opening the monitor.

**No data lines (only heartbeat)**
- Confirm the BNO086 PS0/PS1 straps match `IMU_USART_MODE` (`IMU_MODE_RVC` = PS0 LOW, PS1 LOW).
- Confirm PB6 (MCU TX) connects to BNO086 RX and PB7 (MCU RX) connects to BNO086 TX — UART crosses.
- Check the UART terminal is at 115200 baud.
- UART debug output (`IF_UART`) shows the reset result and any RVC bytes counted at boot.

**UART output is garbled**
- Baud mismatch. Confirm terminal is at 115200.
- If the clock is mis-configured, try `CLOCK_SOURCE_HSE` with the correct `HSE_FREQUENCY_HZ`.

**LED not pulsing**
- Confirm `ENABLE_LED 1` and `LED_PIN PA0` in `config.h`.
- PA0 must not be driven by another peripheral.

**I2C no ACK**
- Confirm the slave is powered and listening at `I2C_OUT_ADDR`.
- Check pull-up resistors on PA8/PA9 (2.2 kΩ to 3.3 V for 400 kHz).
- Reduce `I2C_OUT_SPEED` to `100000` if signal integrity is suspect.
- The endTransmission() return code in `data_publisher.cpp` can be printed to `IF_UART` for diagnosis.

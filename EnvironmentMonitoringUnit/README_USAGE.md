# Environment Monitoring Unit

Reads a **BNO086 IMU** over UART on an **STM32G431KBT6** and serves the latest
reading to a host over **I2C** (the STM32 is the I2C slave). IMU data is also
printed to a debug UART (ST-Link VCP) for monitoring. An LED shows status.

The main loop is non-blocking; each feature is its own module.

## Hardware

- **MCU:** STM32G431KBT6 (32-pin LQFP, 170 MHz capable, runs at 144 MHz by default)
- **Programmer:** external ST-Link over SWD (SWDIO, SWCLK, GND, 3V3)
- **IMU:** BNO086 in UART-SHTP mode

## Connections

| Function | MCU pins | Notes |
|----------|----------|-------|
| Debug UART (`IF_UART`) | PA2 TX / PA3 RX (USART2) | ST-Link VCP, 115200 baud. UART crosses: TX→RX. |
| IMU UART (`IF_IMU`) | PB6 TX / PB7 RX (USART1) | To BNO086. UART crosses: PB6→IMU RX, PB7←IMU TX. |
| I2C slave | PA8 SDA / PA9 SCL (I2C2) | Host reads IMU data here. Address `0x4B`. |
| LED | PA0 | Status indicator. |

### BNO086 control pins (driven by firmware)

| Pin | MCU | Role |
|-----|-----|------|
| NRST | PB4 | Reset (pulsed at boot) |
| BOOTN | PA1 | Held high for normal operation |
| H_INTN | PB5 | Data-ready (active low) |
| CLKSEL0 | PA15 | Held high for external clock |

## How it works

1. At boot the firmware resets the BNO086 and reads its Product ID, printing the
   firmware/part info to the debug UART.
2. The IMU streams readings (~100 Hz). The firmware caps publishing to
   `PUBLISH_RATE_HZ` (default 20 Hz) and stores the latest 14-byte packet.
3. A host on the I2C bus reads that packet on demand — the STM32 never initiates
   transfers. The same packet is also printed to the debug UART.

### LED status

- Boot: a rapid blink burst confirms power-up, then solid for ~1 s once IMU data
  is confirmed.
- Running: a short pulse on each I2C read from the host.

## Build and flash

```bash
pio run -t upload     # or use the VS Code PlatformIO extension
```

Open the ST-Link VCP at **115200 baud** to watch IMU data and boot messages.

## Configuration

All settings live in [`include/config.h`](include/config.h) (IMU-specific
settings in [`include/imu_config.h`](include/imu_config.h)). Common knobs:

| Setting | Default | Meaning |
|---------|---------|---------|
| `SYSCLK_HZ` | `SYSCLK_144MHZ` | CPU speed (64 / 128 / 144 / 170 MHz). |
| `CLOCK_SOURCE` | `CLOCK_SOURCE_HSI` | Internal oscillator (no crystal) or `HSE`. |
| `BAUD_RATE` | `115200` | Debug UART baud. |
| `I2C_SLAVE_ADDR` | `0x4B` | 7-bit I2C address the host reads from. |
| `PUBLISH_RATE_HZ` | `20` | Max publish rate; `0` = every frame. |
| `ENABLE_LED` | `1` | Set `0` to compile the LED out. |
| `IMU_USART_MODE` | `IMU_MODE_SHTP` | Must match the BNO086's PS0/PS1 straps. |

> The debug UART and IMU UART must use different peripherals, and the IMU's
> USART2/PB3-PB4 option clashes with NRST on PB4 — both are caught by a
> compile-time `#error`. Keep the IMU on USART1 (PB6/PB7).

## Modules

| File | Responsibility |
|------|----------------|
| [`include/config.h`](include/config.h) | All user configuration |
| [`src/clock_config.cpp`](src/clock_config.cpp) | System clock setup |
| [`src/bno086.cpp`](src/bno086.cpp) | IMU reset, init, and frame parsing |
| [`src/interfaces.cpp`](src/interfaces.cpp) | The serial objects and `InterfaceId` registry |
| [`src/router.cpp`](src/router.cpp) | Optional any→any UART echo (`ROUTES[]`) |
| [`src/led_indicator.cpp`](src/led_indicator.cpp) | Non-blocking LED status |
| [`src/main.cpp`](src/main.cpp) | Wires modules together; ticks each in `loop()` |

## Troubleshooting

- **No debug output:** terminal at **115200** on the ST-Link VCP; PA2 (TX) reaches the adapter RX.
- **No IMU data:** check PB6/PB7 are wired crossed and `IMU_USART_MODE` matches the board straps.
- **No I2C data:** confirm the host reads address `0x4B` on PA8 (SDA) / PA9 (SCL).
- **Garbled characters:** baud mismatch — try `CLOCK_SOURCE_HSE` with the correct `HSE_FREQUENCY_HZ`.

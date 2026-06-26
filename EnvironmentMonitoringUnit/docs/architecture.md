# Firmware Architecture

## Overview

The Environment Monitoring Unit firmware runs on an **STM32G431KBT6** (32-pin LQFP, 170 MHz, 32 KB RAM, 128 KB Flash) using the Arduino framework via PlatformIO.

The design rule throughout is **non-blocking by default**: `loop()` runs as fast as the MCU allows and each module gets a bounded slice of each tick. No module ever calls `delay()` or spins waiting on hardware — any wait is bounded by a timeout or a byte budget.

---

## Module Map

| Module | Files | Responsibility |
|--------|-------|----------------|
| Configuration | `include/config.h`, `include/imu_config.h` | Single source of truth for every compile-time knob |
| Clock | `src/clock_config.cpp` | PLL setup — overrides the weak core `SystemClock_Config()` to honour `SYSCLK_HZ` |
| Interfaces | `src/interfaces.cpp` / `include/interfaces.h` | Owns the three serial objects; maps `InterfaceId → Stream*` |
| BNO086 bring-up | `src/bno086.cpp` / `include/bno086.h` | Resets the IMU, drives strapping pins, queries Product ID (SHTP mode only) |
| IMU reader | `src/imu_reader.cpp` / `include/imu_reader.h` | State-machine parser for the BNO086 UART-RVC 19-byte stream |
| IMU packet | `include/imu_packet.h` | Shared 13-byte packed struct (`ImuPacket`) produced by `imu_reader` |
| Data publisher | `src/data_publisher.cpp` / `include/data_publisher.h` | Sends each `ImuPacket` as text to USB (if connected) and as binary over I2C (always) |
| Router | `src/router.cpp` / `include/router.h` | Generic any-to-any byte echo via `ROUTES[]`; not used for IMU data |
| Heartbeat | `src/debug_tx.cpp` / `include/debug_tx.h` | Periodic incrementing counter to `HEARTBEAT_DESTINATIONS` |
| LED | `src/led_indicator.cpp` / `include/led_indicator.h` | Non-blocking pulse on heartbeat |
| Main | `src/main.cpp` | Wires modules together; ticks each in `loop()` |

---

## Interfaces

Three serial transports are compiled into every build and started at boot. All generic code refers to them by `InterfaceId` and receives a `Stream*` from `interface_get()`.

| `InterfaceId` | Transport | Pins | Baud | Purpose |
|---------------|-----------|------|------|---------|
| `IF_USB` | Native USB CDC | PA11 (D−) / PA12 (D+) | — (CDC) | Host serial monitor |
| `IF_UART` | USART2 (default) | PA2 TX / PA3 RX | 115200 | Debug console via ST-Link VCP |
| `IF_IMU` | USART1 (default) | PB6 TX / PB7 RX | 115200 | BNO086 UART-RVC stream |

USB runs crystal-less via HSI48 + CRS; no external 48 MHz crystal is required.

---

## Data Flow

```
BNO086 (UART-RVC, 100 Hz)
        │
        │ raw bytes on IF_IMU (USART1 PB6/PB7)
        ▼
 ┌─────────────────┐
 │   imu_reader    │  state-machine parser, validates checksum
 └────────┬────────┘
          │ ImuPacket (13 bytes, 100 Hz)
          ▼
 ┌─────────────────────────────────────────────────┐
 │               data_publisher                    │
 │                                                 │
 │  if (USB host connected)                        │
 │      USB_SERIAL → CSV text line                 │
 │                                                 │
 │  always                                         │
 │      I2C master → slave @ I2C_OUT_ADDR          │
 │      (PA8 SDA / PA9 SCL, I2C2, 400 kHz)         │
 └─────────────────────────────────────────────────┘
```

---

## Main Loop

```cpp
void loop() {
    debug_tx_update();       // heartbeat counter → HEARTBEAT_DESTINATIONS

    router_update();         // byte echo per ROUTES[] (IMU routes removed)

    imu_reader_update();     // drain IF_IMU UART, feed state machine

    ImuPacket pkt;
    if (imu_reader_get(pkt)) {
        data_publisher_publish(pkt);   // USB text + I2C binary
    }

    led_update();            // turn LED off when pulse elapses
}
```

Every call is bounded:
- `router_update()` moves at most 64 bytes per route per tick.
- `imu_reader_update()` drains whatever bytes are in the UART FIFO.
- `data_publisher_publish()` blocks only for the I2C transmission (≈ 0.3 ms at 400 kHz for 13 bytes).

---

## Setup Sequence

```
interfaces_begin()       start USB CDC, USART2 (debug), USART1 (IMU)
│
├─ wait up to 2 s for USB host enumeration (skipped if no USB)
│
led_setup()
router_setup()
bno086_begin()           drive NRST/BOOTN/CLKSEL0, wait for H_INTN,
│                        query Product ID (SHTP only), print to IF_UART
imu_reader_setup()       initialise parser state machine
data_publisher_setup()   Wire.begin() on I2C output bus
debug_tx_setup()         print boot banner, record start time
```

---

## Adding a Module

1. Create `include/feature.h` declaring `feature_setup()` and `feature_update()`.
2. Create `src/feature.cpp` implementing them.
3. `#include "feature.h"` in `src/main.cpp`.
4. Call `feature_setup()` from `setup()` and `feature_update()` from `loop()`.
5. If the module needs to read from an interface, obtain a `Stream*` via `interface_get(id)`.

New transports (SPI, second I2C, etc.) can be added by extending the `InterfaceId` enum and the `interfaces.cpp` registry; the router and heartbeat will route to them automatically.

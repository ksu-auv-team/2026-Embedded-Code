# Hardware Reference

## MCU

**STM32G431KBT6** — 32-pin LQFP "K" package  
170 MHz Cortex-M4F · 32 KB RAM · 128 KB Flash · FPU · DSP

---

## Pin Assignment

| Pin | Direction | Function | Module |
|-----|-----------|----------|--------|
| PA0 | OUT | LED (active high) | `led_indicator` |
| PA1 | OUT | BNO086 BOOTN — held HIGH for normal operation | `bno086` |
| PA2 | OUT | USART2 TX → ST-Link VCP RX | `interfaces` (IF_UART) |
| PA3 | IN | USART2 RX ← ST-Link VCP TX | `interfaces` (IF_UART) |
| PA8 | I/O | I2C2 SDA — output bus to downstream slave | `data_publisher` |
| PA9 | I/O | I2C2 SCL — output bus to downstream slave | `data_publisher` |
| PA11 | I/O | USB D− | `interfaces` (IF_USB) |
| PA12 | I/O | USB D+ | `interfaces` (IF_USB) |
| PA15 | OUT | BNO086 CLKSEL0 — LOW = on-board crystal | `bno086` |
| PB4 | OUT | BNO086 NRST — pulsed LOW ≥ 10 ms at boot | `bno086` |
| PB5 | IN | BNO086 H_INTN — active-LOW ready/interrupt | `bno086` |
| PB6 | OUT | USART1 TX → BNO086 RX | `interfaces` (IF_IMU) |
| PB7 | IN | USART1 RX ← BNO086 TX | `interfaces` (IF_IMU) |

> SWD pins (SWDIO PA13, SWCLK PA14) are reserved for the ST-Link debugger and should not be repurposed during development.

---

## BNO086 Wiring

The BNO086 connects to the MCU via UART (not I2C or SPI). Wire **TX → RX** and **RX ← TX** — UART lines always cross.

```
MCU PB6 (TX)  ────────►  BNO086 RX
MCU PB7 (RX)  ◄────────  BNO086 TX
MCU GND       ──────────  BNO086 GND
MCU 3V3       ──────────  BNO086 VCC

MCU PB4       ──────────  BNO086 NRST
MCU PA1       ──────────  BNO086 BOOTN
MCU PB5       ◄────────  BNO086 H_INTN
MCU PA15      ──────────  BNO086 CLKSEL0
```

### Protocol strapping (PS0 / PS1)

The BNO086 protocol is hardware-selected via PS0 and PS1 (sampled at reset):

| PS1 | PS0 | Protocol |
|-----|-----|----------|
| 0 | 0 | UART-RVC (firmware default) |
| 0 | 1 | UART-SHTP |
| 1 | 0 | I2C |
| 1 | 1 | SPI |

The firmware is configured for **UART-RVC** (`IMU_USART_MODE = IMU_MODE_RVC` in `imu_config.h`). Ensure PS0 = LOW and PS1 = LOW on your board.

### BNO086 control pin behaviour at reset

The firmware drives BOOTN and CLKSEL0 **before** releasing NRST, because these strapping pins are sampled on the rising edge of reset:

1. Drive PB4 (NRST) LOW — hold IMU in reset.
2. Drive PA1 (BOOTN) HIGH — select normal firmware (not DFU bootloader).
3. Drive PA15 (CLKSEL0) LOW — use on-board crystal.
4. Wait ≥ 10 ms.
5. Release PB4 (NRST) HIGH — straps sampled, IMU boots.
6. Wait for PB5 (H_INTN) to assert LOW — device ready.

---

## I2C Output Bus

The MCU acts as **I2C master** on this bus. A downstream device (Raspberry Pi, another MCU, display controller, etc.) acts as slave.

| Signal | MCU pin | I2C role |
|--------|---------|----------|
| SDA | PA8 | Data |
| SCL | PA9 | Clock |

- **Peripheral:** I2C2
- **Speed:** 400 kHz (fast mode) — set `I2C_OUT_SPEED` in `config.h`
- **Slave address:** `0x42` — set `I2C_OUT_ADDR` in `config.h`

I2C lines require pull-up resistors to VCC. Typical values: **4.7 kΩ** for 100 kHz, **2.2 kΩ** for 400 kHz.

The packet sent over I2C on every new IMU frame is the raw `ImuPacket` struct (13 bytes). See [data-formats.md](data-formats.md) for the byte layout.

---

## ST-Link / SWD (Programming & Debugging)

| ST-Link signal | MCU pin |
|----------------|---------|
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| 3V3 | 3V3 (or power the board separately) |

Upload and debug via `pio run -t upload` or the PlatformIO IDE extension.

---

## Power

The MCU runs at **3.3 V**. The BNO086 also operates at 3.3 V. Ensure the I2C output bus slave shares the same ground reference.

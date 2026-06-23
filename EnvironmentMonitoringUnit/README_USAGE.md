# STM32G431KBT6 Environment Monitoring Unit

A small, reliable serial environment monitoring unit built on the Arduino framework. It
exposes three serial interfaces, can echo any of them to any other, transmits
an incrementing counter ("heartbeat") to chosen interfaces, and flashes an LED.
The main loop is fully non-blocking and every feature is a self-contained module.

## Target Hardware

- **MCU:** STM32G431KBT6 — 32-pin LQFP ("K") package, 32K RAM / 128K Flash, 170 MHz
- **Programmer:** external ST-Link over SWD
- **LED:** PA0 (100 ms pulse per transmission)

> `platformio.ini` starts from the generic G431CB board and overrides the MCU and
> variant to the correct 32-pin K-package pinout — a bare-board config, no Nucleo
> assumptions.

## The three interfaces

All three are compiled into every build and started at boot. Generic code refers
to them by `InterfaceId` (`config.h`); the registry in
[`src/interfaces.cpp`](src/interfaces.cpp) maps each id to a `Stream`.

| Id | Transport | Pins | Default baud | Purpose |
|----|-----------|------|--------------|---------|
| `IF_USB` | Native USB CDC | PA11 (D-) / PA12 (D+) | n/a | MCU's own USB COM port |
| `IF_UART` | Debug UART (`UART_SELECT`) | PA2 TX / PA3 RX (USART2) | 9600 | Debug console → ST-Link VCP |
| `IF_IMU` | IMU UART (`IMU_UART_SELECT`) | PB6 TX / PB7 RX (USART1) | 115200 | Permanently attached IMU |

Native USB runs crystal-less on this part (internal HSI48 + CRS); no external
48 MHz crystal is required. PA11=D-, PA12=D+ must reach a USB connector.

## BNO086 IMU

The IMU on `IF_IMU` is a **BNO086** in **UART-SHTP** mode. At startup,
[`src/bno086.cpp`](src/bno086.cpp) resets it and queries its Product ID, then
prints the decoded firmware/part/build info to the debug console (`IF_UART`):

```
BNO086: reset, ready=yes
BNO086 Product ID:
  reset cause: 1
  SW version:  3.7.12
  part number: 10003608
  build number: 370
```

Control pins (driven by firmware):

| Pin | MCU | Role |
|-----|-----|------|
| NRST | PB4 | Reset (active low; pulsed ≥10 ms at boot) |
| BOOTN | PA1 | Bootloader select — held **high** for normal operation |
| H_INTN | PB5 | Host interrupt / data-ready (active **low**) — waited on after reset |
| CLKSEL0 | PA15 | Clock select — held **high** for external clock on the CLK pin |

BOOTN and CLKSEL0 are strapping pins sampled at the rising edge of reset, so
the firmware drives them **before** releasing NRST. The "get info" command is the
SHTP **Product ID Request** (report `0xF9`), wrapped in a UART-SHTP RFC1662 frame
(`0x7E` flags, `0x7D` escaping); the device replies with a Product ID Response
(`0xF8`). The IMU must be strapped for UART-SHTP (PS0/PS1) for this to work — in
UART-RVC mode there is no command channel.

> Note: the `IMU_SEL_USART2_PB3_PB4` option collides with NRST on PB4 and is
> blocked by a compile-time `#error`. Keep the IMU on its USART1 pins.

## Echo routing (any interface → any interface)

Bytes received on one interface can be forwarded to another. Routes are listed
in the `ROUTES[]` table in [`src/router.cpp`](src/router.cpp) as `{from, to}`
pairs. The default echoes the **IMU to the debug UART** (ST-Link VCP):

```cpp
static const Route ROUTES[] = {
    { IF_IMU, IF_UART },   // IMU data -> debug UART (ST-Link VCP)
};
```

Add/remove entries to route any direction, e.g. `{ IF_IMU, IF_USB }` to also
stream the IMU to the USB host, or `{ IF_USB, IF_IMU }` to send host input down
to the IMU.
Each route is non-blocking and bounded (64 bytes/route/tick) so no busy link can
starve the loop. Routes whose endpoints aren't available in the build are skipped.

## Quick Start

1. Wire an ST-Link to the board's SWD pins (SWDIO, SWCLK, GND, 3V3).
2. For UART output: connect MCU **PA2 (TX) → adapter RX**, MCU **PA3 (RX) → adapter TX**,
   common GND. (UART must cross — TX to RX.)
3. For USB output / echo: connect PA11/PA12 to a USB port.
4. IMU: connect the IMU's RX → MCU **PB6 (TX)**, IMU's TX → MCU **PB7 (RX)**,
   common GND (UART crosses). Set `IMU_BAUD_RATE` to match the IMU.
5. Build and flash:
   ```bash
   pio run -t upload     # or use the VS Code PlatformIO extension
   ```
6. Open the relevant terminal. On the UART (9600 baud) you should see:
   ```
   Environment Monitoring Unit Started
   Heartbeat every 1000 ms, counter 0-255
   0
   1
   2
   ...
   ```
   The LED on PA0 pulses (100 ms) with each line.

## Configuration

All knobs live in [`include/config.h`](include/config.h), grouped into sections.

### 1. System clock — `CLOCK_SOURCE` + `SYSCLK_HZ`

**Source** (`CLOCK_SOURCE`): the oscillator feeding the PLL.

| Value | Meaning |
|---|---|
| `CLOCK_SOURCE_HSI` *(default)* | Internal 16 MHz oscillator. No crystal needed. |
| `CLOCK_SOURCE_HSE` | External crystal. Set `HSE_FREQUENCY_HZ` (8/16/24 MHz). |

**Speed** (`SYSCLK_HZ`): the frequency instructions actually execute at. The
source is divided to a 4 MHz PLL input, then multiplied back up to:

| Value | SYSCLK | Notes |
|---|---|---|
| `SYSCLK_170MHZ` *(default)* | 170 MHz | Chip maximum / best performance |
| `SYSCLK_144MHZ` | 144 MHz | Slightly lower power / EMI |
| `SYSCLK_128MHZ` | 128 MHz | Lower power; one fewer flash wait-state |
| `SYSCLK_64MHZ` | 64 MHz | Low power |

[`src/clock_config.cpp`](src/clock_config.cpp) always overrides the core clock
setup, so `SYSCLK_HZ` is honored on **both** HSI and HSE, with verified PLL
multipliers, flash wait-states and voltage scaling per target. `millis()`,
`delay()` and UART baud rates track the real clock automatically (the HAL
recomputes `SystemCoreClock` and derives baud from the live peripheral clock),
so they stay correct at any SYSCLK.

USB CDC is unaffected — it always runs from the separate 48 MHz HSI48, so HSE
need not be a USB-grade crystal and lowering SYSCLK does not break USB.

### 2. Debug UART selection — `UART_SELECT`

| Value | Peripheral | TX / RX |
|---|---|---|
| `UART_SEL_USART2_PA2_PA3` *(default)* | USART2 | PA2 / PA3 (ST-Link VCP) |
| `UART_SEL_LPUART1_PA2_PA3` | LPUART1 | PA2 / PA3 |
| `UART_SEL_USART1_PB6_PB7` | USART1 | PB6 / PB7 — **conflicts with the IMU** |

`BAUD_RATE` (default 9600) applies to `IF_UART`; USB CDC ignores it. The IMU
permanently owns USART1/PB6/PB7, so selecting `UART_SEL_USART1_PB6_PB7` triggers
a compile-time `#error`.

### 3. IMU — [`include/imu_config.h`](include/imu_config.h)

All BNO086 settings live in their own file, `imu_config.h`:

| Setting | Default | Meaning |
|---|---|---|
| `IMU_USART_MODE` | `IMU_MODE_SHTP` | `SHTP` (bidirectional, get-info) or `RVC` (one-way stream) — must match the board's PS0/PS1 straps |
| `IMU_BAUD_RATE` | *derived* | **Auto from mode: SHTP → 3 000 000, RVC → 115 200.** Override only if needed |
| `IMU_CLOCK` | `IMU_CLOCK_EXTERNAL` | `EXTERNAL` (CLKSEL0 high, CLK pin driven) or `CRYSTAL` (CLKSEL0 low) |
| `IMU_UART_SELECT` | `IMU_SEL_USART1_PB6_PB7` | Peripheral + pins (see below) |

`IMU_UART_SELECT` options:

| Value | Peripheral | TX / RX |
|---|---|---|
| `IMU_SEL_USART1_PB6_PB7` *(default)* | USART1 | PB6 / PB7 |
| `IMU_SEL_USART1_PA9_PA10` | USART1 | PA9 / PA10 |
| `IMU_SEL_USART2_PB3_PB4` | USART2 | PB3 / PB4 — blocked (clashes NRST/PB4) |

> **Baud caveat:** UART-SHTP runs at **3 Mbaud**, UART-RVC at **115200** — they
> are *not* interchangeable. Deriving the baud from `IMU_USART_MODE` keeps them
> in sync automatically. The debug UART (`UART_SELECT`) and IMU UART must use
> different peripherals — a collision is a compile-time `#error`.

### 4. Echo routing — `ROUTES[]`

Edit the `{from, to}` table in [`src/router.cpp`](src/router.cpp). See *Echo
routing* above. Default: `{ IF_IMU, IF_UART }` (IMU → ST-Link VCP). An empty
table disables all echoing.

### 5. Heartbeat — `HEARTBEAT_DESTINATIONS`

Set `HEARTBEAT_DESTINATIONS` in `config.h` to the interface(s) that receive the
counter (default `IF_USB`). Examples: `IF_UART`, or `IF_USB, IF_UART` for both;
leave empty to disable. Also `SEND_INTERVAL_MS` (default 1000) and `COUNTER_MAX`
(default 255).

### 6. LED — `ENABLE_LED`

`ENABLE_LED` (default 1; `0` compiles the feature out), `LED_PIN` (default PA0),
`LED_PULSE_MS` (default 100).

## Architecture

Each feature is a module with a `*_setup()` and a non-blocking `*_update()`:

| File | Responsibility |
|------|----------------|
| [`include/config.h`](include/config.h) | All user configuration (single source of truth) |
| [`src/clock_config.cpp`](src/clock_config.cpp) | HSE clock override (HSI uses the core default) |
| [`src/interfaces.cpp`](src/interfaces.cpp) / [`.h`](include/interfaces.h) | Owns the 3 serial objects; `InterfaceId → Stream*` registry |
| [`src/router.cpp`](src/router.cpp) / [`.h`](include/router.h) | Generic any→any echo via `ROUTES[]` |
| [`src/debug_tx.cpp`](src/debug_tx.cpp) / [`.h`](include/debug_tx.h) | Periodic heartbeat to `HEARTBEAT_DESTINATIONS[]` |
| [`src/led_indicator.cpp`](src/led_indicator.cpp) / [`.h`](include/led_indicator.h) | Non-blocking LED pulse |
| [`src/main.cpp`](src/main.cpp) | Wires modules together; ticks each in `loop()` |

The main loop never blocks:

```cpp
void loop() {
    debug_tx_update();   // heartbeat to configured interface(s)
    router_update();     // echo interfaces per ROUTES[]
    led_update();        // turn LED off when its pulse elapses
}
```

## Troubleshooting

**No UART output:**
- Confirm the terminal is at **9600 baud** on the ST-Link VCP port.
- Confirm MCU PA2 (TX) reaches the adapter **RX** input (UART crosses — TX to RX).

**No USB output / echo:**
- Confirm the board enumerated as a COM port (Device Manager / `dmesg`).
- PA11=D-, PA12=D+ must reach the USB connector; baud is irrelevant.
- For an echo, confirm the relevant `{from, to}` entry is in `ROUTES[]`
  (`src/router.cpp`) and both endpoints are wired.

**No IMU data on USB:**
- Confirm `{ IF_IMU, IF_USB }` is in `ROUTES[]`.
- Confirm `IMU_BAUD_RATE` matches the IMU and PB6/PB7 are wired crossed.

**LED not pulsing:**
- Check `ENABLE_LED` is `1` and `LED_PIN` matches your wiring.
- PA0 must not be reused by another peripheral.

**Garbled characters (not silence):**
- Baud mismatch or clock issue. Try `CLOCK_SOURCE_HSE` with the correct
  `HSE_FREQUENCY_HZ` for better baud accuracy.

## Extending

The module pattern makes it easy to add features (e.g. parse IMU frames instead
of raw echo, or a command interface): create `feature.h/.cpp` with
`feature_setup()` / `feature_update()`, then call them from `setup()` and
`loop()` in `main.cpp`. New interfaces just need an `InterfaceId` and an entry
in the `interfaces.cpp` registry to become routable.

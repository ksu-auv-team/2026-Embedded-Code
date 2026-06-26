# Configuration Reference

All user-facing knobs live in two files:

- **`include/config.h`** — system clock, UART, routing, heartbeat, LED, I2C output
- **`include/imu_config.h`** — BNO086-specific settings (mode, baud, clock, pins)

Edit these files to reconfigure the firmware without touching any source code.

---

## `include/config.h`

### Section 1 — System Clock

```c
#define CLOCK_SOURCE   CLOCK_SOURCE_HSI   // or CLOCK_SOURCE_HSE
#define HSE_FREQUENCY_HZ  16000000UL      // only used when CLOCK_SOURCE == HSE
#define SYSCLK_HZ      SYSCLK_170MHZ
```

**`CLOCK_SOURCE`**

| Value | Description |
|-------|-------------|
| `CLOCK_SOURCE_HSI` *(default)* | Internal 16 MHz RC oscillator. No crystal required. |
| `CLOCK_SOURCE_HSE` | External crystal/oscillator. Set `HSE_FREQUENCY_HZ` to 8, 16, or 24 MHz. Better baud-rate accuracy. |

USB always sources its 48 MHz from the independent HSI48 oscillator regardless of `CLOCK_SOURCE`.

**`SYSCLK_HZ`**

| Macro | Frequency | Flash wait-states | Notes |
|-------|-----------|-------------------|-------|
| `SYSCLK_170MHZ` *(default)* | 170 MHz | 4 | Chip maximum |
| `SYSCLK_144MHZ` | 144 MHz | 4 | |
| `SYSCLK_128MHZ` | 128 MHz | 3 | One fewer wait-state |
| `SYSCLK_64MHZ` | 64 MHz | 1 | Low power |

`millis()`, `delay()`, and UART baud rates all track the real clock automatically — they stay correct at any `SYSCLK_HZ`.

---

### Section 2 — Debug UART

```c
#define UART_SELECT   UART_SEL_USART2_PA2_PA3
#define BAUD_RATE     115200
```

**`UART_SELECT`**

| Value | Peripheral | TX / RX pins |
|-------|-----------|--------------|
| `UART_SEL_USART2_PA2_PA3` *(default)* | USART2 | PA2 / PA3 (ST-Link VCP) |
| `UART_SEL_LPUART1_PA2_PA3` | LPUART1 | PA2 / PA3 |
| `UART_SEL_USART1_PB6_PB7` | USART1 | PB6 / PB7 — **blocked**: conflicts with IMU |

The IMU permanently owns USART1/PB6/PB7. Selecting `UART_SEL_USART1_PB6_PB7` triggers a compile-time `#error`.

**`BAUD_RATE`** — applies to `IF_UART` only; USB CDC ignores it.

---

### Section 3 — IMU

IMU settings are delegated to `include/imu_config.h`. See [imu_config.h section](#includeimuconfigh) below.

---

### Section 4 — Echo Routing

The routing table is edited directly in `src/router.cpp`:

```cpp
static const Route ROUTES[] = {
    // { from, to },
};
```

Each `{from, to}` entry forwards bytes received on `from` out to `to`. The table is empty by default because `imu_reader` now owns `IF_IMU` bytes — do not add `IF_IMU` as a source here or bytes will be split between the router and the parser.

Permitted additions:

```cpp
{ IF_UART, IF_USB },   // mirror debug UART onto USB host
{ IF_USB,  IF_UART },  // pipe USB host input to debug UART
```

Each route moves at most 64 bytes per tick to prevent one busy link from starving the loop.

---

### Section 5 — Heartbeat

```c
#define HEARTBEAT_DESTINATIONS  IF_USB
#define SEND_INTERVAL_MS        1000
#define COUNTER_MAX             255
```

**`HEARTBEAT_DESTINATIONS`** — one or more `InterfaceId` values:

```c
#define HEARTBEAT_DESTINATIONS  IF_USB            // USB only (default)
#define HEARTBEAT_DESTINATIONS  IF_UART           // debug UART only
#define HEARTBEAT_DESTINATIONS  IF_USB, IF_UART   // both
// leave empty to disable the heartbeat
```

**`SEND_INTERVAL_MS`** — interval between counter transmissions (ms).  
**`COUNTER_MAX`** — counter wraps at this value (0–255).

---

### Section 6 — LED Indicator

```c
#define ENABLE_LED    1
#define LED_PIN       PA0
#define LED_PULSE_MS  100
```

Set `ENABLE_LED` to `0` to compile the LED feature out entirely (saves a few bytes and a `pinMode` call).

---

### Section 7 — I2C Output Bus

```c
#define I2C_OUT_SDA    PA8
#define I2C_OUT_SCL    PA9
#define I2C_OUT_ADDR   0x42
#define I2C_OUT_SPEED  400000
```

| Macro | Default | Description |
|-------|---------|-------------|
| `I2C_OUT_SDA` | `PA8` | SDA pin (I2C2) |
| `I2C_OUT_SCL` | `PA9` | SCL pin (I2C2) |
| `I2C_OUT_ADDR` | `0x42` | 7-bit slave address of the downstream device |
| `I2C_OUT_SPEED` | `400000` | Bus speed in Hz — `100000` (standard) or `400000` (fast) |

The MCU transmits a 13-byte `ImuPacket` to `I2C_OUT_ADDR` on every parsed BNO086 frame (~100 Hz). See [data-formats.md](data-formats.md).

---

## `include/imu_config.h`

### Section 1 — UART Mode

```c
#define IMU_USART_MODE  IMU_MODE_RVC
```

| Value | Protocol | Baud | Direction |
|-------|----------|------|-----------|
| `IMU_MODE_RVC` *(default)* | UART-RVC | 115200 | One-way streaming: heading + accel at 100 Hz |
| `IMU_MODE_SHTP` | UART-SHTP | 3000000 | Bidirectional: full sensor hub, Product ID query |

**This must match the PS0/PS1 hardware straps on your board.** See [hardware.md](hardware.md#protocol-strapping-ps0--ps1).

In RVC mode the firmware parses frames in `imu_reader` and publishes data via `data_publisher`. In SHTP mode only a Product ID query is performed at startup; continuous data readout from SHTP is not yet implemented.

### Section 2 — Baud Rate

```c
// #define IMU_BAUD_RATE  <value>   // uncomment to override
```

Auto-derived from `IMU_USART_MODE`: RVC → 115200, SHTP → 3000000. Override only if your board reconfigures the baud.

### Section 3 — Clock Source

```c
#define IMU_CLOCK  IMU_CLOCK_CRYSTAL
```

| Value | CLKSEL0 | Description |
|-------|---------|-------------|
| `IMU_CLOCK_CRYSTAL` *(default)* | LOW | BNO086 uses its on-board crystal or internal oscillator |
| `IMU_CLOCK_EXTERNAL` | HIGH | BNO086 clocks from an external signal on the CLK pin |

### Section 4 — IMU UART Pins

```c
#define IMU_UART_SELECT  IMU_SEL_USART1_PB6_PB7
```

| Value | Peripheral | TX / RX |
|-------|-----------|---------|
| `IMU_SEL_USART1_PB6_PB7` *(default)* | USART1 | PB6 / PB7 |
| `IMU_SEL_USART1_PA9_PA10` | USART1 | PA9 / PA10 |
| `IMU_SEL_USART2_PB3_PB4` | USART2 | PB3 / PB4 — **blocked**: PB4 = BNO086 NRST |

---

## Compile-time Guards

The following collisions are detected at compile time with `#error`:

| Condition | Error message |
|-----------|---------------|
| Debug UART and IMU UART share a USART peripheral | `"UART_SELECT and IMU_UART_SELECT use the same peripheral."` |
| IMU selected on PB3/PB4 | `"IMU_SEL_USART2_PB3_PB4 conflicts with BNO086 NRST on PB4."` |

#include "interfaces.h"

/**
 * Concrete serial objects.
 *
 * Each HardwareSerial is constructed (RX, TX) so the Arduino core resolves the
 * peripheral from the variant pin map:
 *   - Debug UART: per UART_SELECT (config.h).
 *   - IMU UART:   USART1 on PB6 (TX) / PB7 (RX), fixed.
 */

/* --- Debug UART (IF_UART), selected by UART_SELECT --- */
#if UART_SELECT == UART_SEL_USART2_PA2_PA3
static HardwareSerial uart_debug(PA_3_ALT1, PA_2_ALT1);  // USART2
#elif UART_SELECT == UART_SEL_LPUART1_PA2_PA3
static HardwareSerial uart_debug(PA3, PA2);              // LPUART1
#else
#error "Invalid UART_SELECT in config.h"
#endif

/* --- IMU UART (IF_IMU), selected by IMU_UART_SELECT --- */
#if IMU_UART_SELECT == IMU_SEL_USART1_PB6_PB7
static HardwareSerial uart_imu(PB7, PB6);                // USART1 (RX, TX)
#elif IMU_UART_SELECT == IMU_SEL_USART1_PA9_PA10
static HardwareSerial uart_imu(PA10, PA9);               // USART1
#elif IMU_UART_SELECT == IMU_SEL_USART2_PB3_PB4
static HardwareSerial uart_imu(PB4, PB3);                // USART2
#else
#error "Invalid IMU_UART_SELECT in config.h"
#endif

void interfaces_begin(void) {
    uart_debug.begin(BAUD_RATE);
    /* NOTE: the IMU UART is NOT begun here. It is owned by the 7Semi BNO08x
     * UART bus (see imu_source.cpp), which calls uart_imu.begin(IMU_BAUD_RATE)
     * from imu_source_setup(). Keeping a single owner avoids begin()-ing the
     * same USART twice at potentially different rates. */

#if USB_AVAILABLE
    USB_SERIAL.begin();  /* baud ignored by native USB */
#endif
}

HardwareSerial &imu_hardware_serial(void) {
    return uart_imu;
}

Stream *interface_get(InterfaceId id) {
    switch (id) {
#if USB_AVAILABLE
        case IF_USB:  return &USB_SERIAL;
#else
        case IF_USB:  return nullptr;
#endif
        case IF_UART: return &uart_debug;
        case IF_IMU:  return &uart_imu;
        default:      return nullptr;
    }
}

const char *interface_name(InterfaceId id) {
    switch (id) {
        case IF_USB:  return "USB";
        case IF_UART: return "UART";
        case IF_IMU:  return "IMU";
        default:      return "?";
    }
}

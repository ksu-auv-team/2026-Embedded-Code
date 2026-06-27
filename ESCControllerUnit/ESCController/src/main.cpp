#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Detection-critical I2C settings with safe defaults.
// They can be overridden from PlatformIO build_flags if needed.
#ifndef ESC_I2C_ADDRESS
#define ESC_I2C_ADDRESS 0x4C
#endif

#ifndef ESC_I2C_SDA_PIN
#if defined(PIN_WIRE_SDA)
#define ESC_I2C_SDA_PIN PIN_WIRE_SDA
#else
#define ESC_I2C_SDA_PIN PB11
#endif
#endif

#ifndef ESC_I2C_SCL_PIN
#if defined(PIN_WIRE_SCL)
#define ESC_I2C_SCL_PIN PIN_WIRE_SCL
#else
#define ESC_I2C_SCL_PIN PB10
#endif
#endif

static_assert((ESC_I2C_ADDRESS >= 0x08) && (ESC_I2C_ADDRESS <= 0x77), "ESC_I2C_ADDRESS must be a 7-bit I2C address (0x08..0x77)");

static constexpr uint8_t I2C_SLAVE_ADDRESS = static_cast<uint8_t>(ESC_I2C_ADDRESS);
static constexpr uint8_t THRUST_CMD = 0x00;
static constexpr uint8_t I2C_SDA_PIN = ESC_I2C_SDA_PIN;
static constexpr uint8_t I2C_SCL_PIN = ESC_I2C_SCL_PIN;

static constexpr uint8_t ESC_COUNT = 8;
static constexpr uint16_t PWM_MIN = 1100;
static constexpr uint16_t PWM_MAX = 1900;

static constexpr uint16_t LOOP_FREQUENCY_HZ = 50;
static constexpr uint16_t LOOP_DELAY_MS = 1000 / LOOP_FREQUENCY_HZ;
static constexpr uint32_t LINK_TIMEOUT_MS = 2000;
static constexpr uint32_t LED_SLOW_BLINK_MS = 500;
static constexpr uint32_t LED_FAST_BLINK_MS = 120;

// Matches CubeMX main.h pinout: ESC1..ESC8 = PA1..PA7, PA10.
static const uint8_t ESC_PINS[ESC_COUNT] = {
    PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA10
};

static constexpr uint8_t STATUS_LED_PIN = PA0;
static constexpr size_t PACKET_SIZE = ESC_COUNT + 1;

Servo esc[ESC_COUNT];
TwoWire i2cBus(I2C_SDA_PIN, I2C_SCL_PIN);

volatile uint8_t motorValues[ESC_COUNT];
volatile uint8_t i2cRxPacket[PACKET_SIZE];
volatile uint8_t i2cRxLen = 0;
volatile bool i2cPacketReady = false;

volatile bool newData = false;
volatile bool packetFlashRequest = false;
volatile uint32_t lastReceiveTime = 0;

uint32_t statusLedLastToggleTime = 0;
uint8_t packetFlashTogglesRemaining = 0;
bool statusLedState = false;
uint32_t lastControlTick = 0;

static inline uint16_t mapPwm(uint8_t value) {
    return static_cast<uint16_t>((static_cast<uint32_t>(value) * (PWM_MAX - PWM_MIN)) / 255U + PWM_MIN);
}

static void updateEscOutputs() {
    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        esc[i].writeMicroseconds(mapPwm(motorValues[i]));
    }
}

static void setEscFailsafe() {
    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        motorValues[i] = 128;
        esc[i].writeMicroseconds(mapPwm(128));
    }
}

static void setStatusLed(bool state) {
    statusLedState = state;
    digitalWrite(STATUS_LED_PIN, state ? HIGH : LOW);
}

static void updateStatusLed(uint32_t now) {
    if (packetFlashRequest) {
        packetFlashRequest = false;
        packetFlashTogglesRemaining = 4;
        statusLedLastToggleTime = now;
    }

    if (packetFlashTogglesRemaining > 0) {
        if ((now - statusLedLastToggleTime) >= LED_FAST_BLINK_MS) {
            statusLedLastToggleTime = now;
            setStatusLed(!statusLedState);
            packetFlashTogglesRemaining--;
        }
        return;
    }

    if ((now - lastReceiveTime) <= LINK_TIMEOUT_MS) {
        if ((now - statusLedLastToggleTime) >= LED_SLOW_BLINK_MS) {
            statusLedLastToggleTime = now;
            setStatusLed(!statusLedState);
        }
    } else {
        setStatusLed(false);
    }
}

// I2C packet format: [cmd][8 bytes thrust], where cmd is THRUST_CMD.
static void onI2CReceive(int bytes) {
    if (bytes <= 0) {
        return;
    }

    // Keep ISR short to avoid clock stretching on the I2C bus.
    if (i2cPacketReady) {
        while (i2cBus.available()) {
            i2cBus.read();
        }
        return;
    }

    uint8_t len = 0;

    while (i2cBus.available()) {
        const uint8_t b = static_cast<uint8_t>(i2cBus.read());
        if (len < PACKET_SIZE) {
            i2cRxPacket[len] = b;
        }
        len++;
    }

    if (len == PACKET_SIZE) {
        i2cRxLen = len;
        i2cPacketReady = true;
    }
}

static void onI2CRequest() {
    // Answer read probes quickly so masters/scanners do not wait.
    i2cBus.write(static_cast<uint8_t>(0xA5));
}

static void processI2CPacket(uint32_t now) {
    if (!i2cPacketReady) {
        return;
    }

    uint8_t packet[PACKET_SIZE];
    uint8_t len = 0;

    noInterrupts();
    len = i2cRxLen;
    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        packet[i] = i2cRxPacket[i];
    }
    i2cPacketReady = false;
    interrupts();

    if (len != PACKET_SIZE || packet[0] != THRUST_CMD) {
        return;
    }

    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        motorValues[i] = packet[i + 1];
    }

    newData = true;
    packetFlashRequest = true;
    lastReceiveTime = now;
}

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    setStatusLed(false);

    i2cBus.begin(I2C_SLAVE_ADDRESS);
    i2cBus.onReceive(onI2CReceive);
    i2cBus.onRequest(onI2CRequest);

    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        motorValues[i] = 128;
        esc[i].attach(ESC_PINS[i]);
    }

    updateEscOutputs();
    lastReceiveTime = millis();
    statusLedLastToggleTime = lastReceiveTime;
}

void loop() {
    const uint32_t now = millis();

    processI2CPacket(now);

    if ((now - lastControlTick) >= LOOP_DELAY_MS) {
        lastControlTick = now;

        if ((now - lastReceiveTime) > LINK_TIMEOUT_MS) {
            setEscFailsafe();
        } else if (newData) {
            updateEscOutputs();
            newData = false;
        }

        updateStatusLed(now);
    }
}
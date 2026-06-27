#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

static constexpr uint8_t I2C_SLAVE_ADDRESS = 0x4C;
static constexpr uint8_t THRUST_CMD = 0x00;

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
        while (Wire.available()) {
            Wire.read();
        }
        return;
    }

    uint8_t len = 0;

    while (Wire.available()) {
        const uint8_t b = static_cast<uint8_t>(Wire.read());
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

    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);

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
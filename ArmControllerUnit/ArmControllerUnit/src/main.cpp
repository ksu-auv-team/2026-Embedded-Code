#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

static constexpr uint8_t I2C_ADDRESS = 0x4D;
static constexpr uint8_t PIN_SDA = PIN_WIRE_SDA;
static constexpr uint8_t PIN_SCL = PIN_WIRE_SCL;

static constexpr uint8_t PIN_PWM1 = PA5;
static constexpr uint8_t PIN_PWM2 = PA1;
static constexpr uint8_t PIN_SIGNAL = PA0;

static constexpr uint16_t PWM_MIN = 1100;
static constexpr uint16_t PWM_NEUTRAL = 1500;
static constexpr uint16_t PWM_MAX = 1900;

static constexpr uint8_t NUM_SERVOS = 2;
static constexpr uint8_t REGISTER_THRUST = 0x00;
static constexpr uint8_t PACKET_SIZE = NUM_SERVOS + 1;
static constexpr uint32_t LED_BLINK_INTERVAL_MS = 120;

Servo servos[NUM_SERVOS];
TwoWire i2cBus(PIN_SDA, PIN_SCL);
volatile bool blinkRequest = false;
volatile bool packetReady = false;
volatile uint8_t rxPacket[PACKET_SIZE];
volatile uint8_t rxLength = 0;

uint8_t blinkTogglesRemaining = 0;
bool signalLedState = false;
uint32_t lastBlinkToggleMs = 0;

const uint8_t SERVO_PINS[NUM_SERVOS] = {
  PIN_PWM1, PIN_PWM2
};

static inline uint16_t toMicroseconds(uint8_t value) {
  return static_cast<uint16_t>(map(value, 0, 255, PWM_MIN, PWM_MAX));
}

void onReceive(int bytes) {
  (void)bytes;

  uint8_t len = 0;

  if (bytes > 0) {
    blinkRequest = true;
  }

  if (packetReady) {
    while (i2cBus.available()) {
      i2cBus.read();
    }
    return;
  }

  while (i2cBus.available() && len < PACKET_SIZE) {
    rxPacket[len++] = static_cast<uint8_t>(i2cBus.read());
  }
  while (i2cBus.available()) {
    i2cBus.read();
  }

  if (len == PACKET_SIZE) {
    rxLength = len;
    packetReady = true;
  }
}

static void processI2CPacket() {
  if (!packetReady) {
    return;
  }

  uint8_t packet[PACKET_SIZE];
  uint8_t len = 0;

  noInterrupts();
  len = rxLength;
  for (uint8_t i = 0; i < PACKET_SIZE; i++) {
    packet[i] = rxPacket[i];
  }
  packetReady = false;
  interrupts();

  if (len != PACKET_SIZE || packet[0] != REGISTER_THRUST) {
    return;
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(toMicroseconds(packet[i + 1]));
  }

}

static void updateSignalBlink(uint32_t now) {
  if (blinkRequest) {
    blinkRequest = false;
    blinkTogglesRemaining = 4;
    lastBlinkToggleMs = now;
  }

  if (blinkTogglesRemaining == 0) {
    return;
  }

  if ((now - lastBlinkToggleMs) >= LED_BLINK_INTERVAL_MS) {
    lastBlinkToggleMs = now;
    signalLedState = !signalLedState;
    digitalWrite(PIN_SIGNAL, signalLedState ? HIGH : LOW);
    blinkTogglesRemaining--;
  }
}

void setup() {
  pinMode(PIN_SIGNAL, OUTPUT);
  digitalWrite(PIN_SIGNAL, LOW);

  i2cBus.begin(I2C_ADDRESS);
  i2cBus.onReceive(onReceive);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(PWM_NEUTRAL);
  }
}

void loop() {
  processI2CPacket();
  updateSignalBlink(millis());
}
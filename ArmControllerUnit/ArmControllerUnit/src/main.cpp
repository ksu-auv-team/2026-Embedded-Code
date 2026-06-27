#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

static constexpr uint8_t I2C_ADDRESS = 0x4D;
static constexpr uint8_t PIN_SDA = PA8;
static constexpr uint8_t PIN_SCL = PA9;

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
volatile bool blinkRequest = false;

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

  uint8_t buf[PACKET_SIZE];
  uint8_t len = 0;

  while (Wire.available() && len < PACKET_SIZE) {
    buf[len++] = static_cast<uint8_t>(Wire.read());
  }
  while (Wire.available()) {
    Wire.read();
  }

  if (len != PACKET_SIZE || buf[0] != REGISTER_THRUST) {
    return;
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(toMicroseconds(buf[i + 1]));
  }

  blinkRequest = true;
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

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onReceive);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(PWM_NEUTRAL);
  }
}

void loop() {
  updateSignalBlink(millis());
  delay(10);
}
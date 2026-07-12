#include <Arduino.h>
#include <Wire.h>

// Hall input pin from your STM pin map.
constexpr uint8_t HALL_SENSOR_PIN = PB5;
constexpr bool HALL_ACTIVE_STATE = LOW;

// Orin I2C slave address and packet format.
constexpr uint8_t ORIN_I2C_ADDRESS = 0x08;
constexpr uint8_t KILL_PACKET_MAGIC = 0xA5;
constexpr unsigned long DEBOUNCE_MS = 30;
constexpr unsigned long I2C_HEARTBEAT_MS = 100;

bool lastRawState = HIGH;
bool debouncedState = HIGH;
unsigned long lastChangeMs = 0;
unsigned long lastI2cTxMs = 0;

void sendKillStateI2C(bool killActive) {
  Wire.beginTransmission(ORIN_I2C_ADDRESS);
  Wire.write(KILL_PACKET_MAGIC);
  Wire.write(killActive ? 1 : 0);
  const uint8_t txStatus = Wire.endTransmission();

  if (txStatus == 0) {
    Serial.print("I2C TX OK, kill=");
    Serial.println(killActive ? "1" : "0");
  } else {
    Serial.print("I2C TX FAIL, code=");
    Serial.println(txStatus);
  }
}

void setup() {
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(100);

  // STM32 Arduino defaults I2C to the board's primary SDA/SCL pins.
  Wire.setSDA(PA8);
  Wire.setSCL(PA9);
  Wire.begin();

  Serial.println("Kill switch monitor started");

  // Publish an initial safe state at boot.
  sendKillStateI2C(false);
}

void loop() {
  const bool rawState = digitalRead(HALL_SENSOR_PIN);

  if (rawState != lastRawState) {
    lastRawState = rawState;
    lastChangeMs = millis();
  }

  if ((millis() - lastChangeMs) >= DEBOUNCE_MS && rawState != debouncedState) {
    debouncedState = rawState;

    const bool killActive = (debouncedState == HALL_ACTIVE_STATE);
    sendKillStateI2C(killActive);
    lastI2cTxMs = millis();

    if (killActive) {
      Serial.println("Hall detected -> kill active");
    } else {
      Serial.println("Hall not detected -> kill inactive");
    }
  }

  // Heartbeat helps Orin detect stale communication.
  if ((millis() - lastI2cTxMs) >= I2C_HEARTBEAT_MS) {
    const bool killActive = (debouncedState == HALL_ACTIVE_STATE);
    sendKillStateI2C(killActive);
    lastI2cTxMs = millis();
  }

  delay(2);
}
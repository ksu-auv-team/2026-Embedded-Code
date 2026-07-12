#include <Arduino.h>

// Wiring:
// - A1120 Hall sensor output -> HALL_SENSOR_PIN
// - A1120 GND -> Arduino GND
// - A1120 VCC -> Arduino 5V
// - Jetson notification input <- JETSON_NOTIFY_PIN (consider level shifting to 3.3V)

constexpr uint8_t HALL_SENSOR_PIN = 7;       // Digital input from A1120 output
constexpr uint8_t JETSON_NOTIFY_PIN = 8;     // Digital output to Jetson
constexpr bool HALL_ACTIVE_STATE = LOW;      // A1120/open-drain style: active pulls line LOW
constexpr unsigned long DEBOUNCE_MS = 30;    // Debounce/filter window

bool lastRawState = HIGH;
bool debouncedState = HIGH;
unsigned long lastChangeMs = 0;

void updateJetsonSignal(bool isOffPosition) {
  digitalWrite(JETSON_NOTIFY_PIN, isOffPosition ? HIGH : LOW);
}

void setup() {
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(JETSON_NOTIFY_PIN, OUTPUT);

  // Default: Hall target not detected until proven otherwise.
  updateJetsonSignal(false);

  Serial.begin(115200);
  delay(100);
  Serial.println("Hall sensor monitor started");
}

void loop() {
  const bool rawState = digitalRead(HALL_SENSOR_PIN);

  if (rawState != lastRawState) {
    lastRawState = rawState;
    lastChangeMs = millis();
  }

  if ((millis() - lastChangeMs) >= DEBOUNCE_MS && rawState != debouncedState) {
    debouncedState = rawState;

    // True when magnetic field detection is active.
    const bool hallDetected = (debouncedState == HALL_ACTIVE_STATE);
    updateJetsonSignal(hallDetected);

    if (hallDetected) {
      Serial.println("Hall detected -> Jetson signal HIGH");
    } else {
      Serial.println("Hall not detected -> Jetson signal LOW");
    }
  }

  delay(2);
}
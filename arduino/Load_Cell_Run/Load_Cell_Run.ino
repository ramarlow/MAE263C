// ============================================================
// TAL220 + HX711 — RUNTIME SKETCH
// Loads calibration factor from EEPROM and streams readings
// as fast as the HX711 allows (~10Hz at default, ~80Hz at
// high speed mode if your HX711 board has RATE pin access).
// ============================================================

#include <HX711.h>
#include <EEPROM.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int EEPROM_ADDR = 0;

const int LOADCELL1_DOUT_PIN = 4;
const int LOADCELL1_SCK_PIN = 5;

// Set to 1 for fastest single reading, higher for more stability.
// At 1, you get raw speed (~10Hz). At 4-5, noise drops noticeably.
const int NUM_READINGS = 1;

HX711 scale;
HX711 scale1;
float calibrationFactor = 0.0;

float load = 0, load1 = 0;

void setup() {
  Serial.begin(57600);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  delay(500);

  EEPROM.get(EEPROM_ADDR, calibrationFactor);

  if (calibrationFactor == 0.0 || isnan(calibrationFactor) || calibrationFactor < 0) {
    Serial.println("ERROR: No valid calibration in EEPROM.");
    Serial.println("Upload and run the calibration sketch first.");
    while (true);  // halt
  }

  scale.set_scale(calibrationFactor);
  scale1.set_scale(calibrationFactor);
  scale.tare();
  scale1.tare();

  // Optional header for CSV logging
  Serial.println("grams");
}

void loop() {
  if (scale.is_ready()) {
    load = scale.get_units(NUM_READINGS);
    // Serial.println(scale.get_units(NUM_READINGS), 2);
  }
  if (scale1.is_ready()) {
    load1 = scale1.get_units(NUM_READINGS);
    // Serial.println(scale1.get_units(NUM_READINGS), 2);
  }
  //Serial.print("l1:");
  Serial.print(load,2);
  Serial.print("\t");
  //Serial.print("l2:");
  Serial.println(load1,2);
  // No delay — runs as fast as HX711 conversion allows
}
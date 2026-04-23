// ============================================================
// TAL220 + HX711 — CALIBRATION SKETCH
// Run this once to find and save your calibration factor.
// Then upload the runtime sketch for live measurements.
// ============================================================

#include "HX711.h"
#include <EEPROM.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int EEPROM_ADDR = 0;

HX711 scale;

void flushSerial() {
  delay(100);
  while (Serial.available()) Serial.read();
}

void waitForSerial() {
  Serial.println("(send any key to continue)");
  while (!Serial.available()) delay(50);
  flushSerial();
}

float waitForFloat() {
  flushSerial();
  while (!Serial.available()) delay(50);
  float val = Serial.parseFloat();
  flushSerial();
  return val;
}

void setup() {
  Serial.begin(57600);
  delay(2000);

  Serial.println("\n=== TAL220 Calibration ===");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(500);

  // --- Step 1: Tare ---
  Serial.println("\nStep 1: Remove all load from the cell.");
  waitForSerial();
  scale.set_scale();
  scale.tare();
  Serial.println("Tare complete.\n");

  // --- Step 2: Known weight ---
  Serial.println("Step 2: Place a known weight on the cell.");
  Serial.println("Type the weight in grams and press Send:");
  float knownWeight = waitForFloat();

  if (knownWeight <= 0.0) {
    Serial.println("Invalid weight entered. Please re-upload and try again.");
    while (true);  // halt
  }

  Serial.print("Known weight: ");
  Serial.print(knownWeight, 2);
  Serial.println(" g");

  // --- Step 3: Read and compute factor ---
  Serial.println("Reading... keep weight still.");
  float rawReading = scale.get_units(20);
  float calibrationFactor = rawReading / knownWeight;

  Serial.print("Calibration factor: ");
  Serial.println(calibrationFactor, 4);
  scale.set_scale(calibrationFactor);

  // --- Step 4: Verify ---
  Serial.println("\nVerification — keep weight on cell:");
  float verified = scale.get_units(10);
  Serial.print("Reading:  ");
  Serial.print(verified, 2);
  Serial.println(" g");
  Serial.print("Expected: ");
  Serial.print(knownWeight, 2);
  Serial.println(" g");

  float error = abs(verified - knownWeight) / knownWeight * 100.0;
  Serial.print("Error: ");
  Serial.print(error, 2);
  Serial.println(" %");

  // --- Step 5: Save ---
  EEPROM.put(EEPROM_ADDR, calibrationFactor);
  Serial.println("\nCalibration factor saved to EEPROM.");
  Serial.println("Upload the runtime sketch to begin measuring.");
}

void loop() {
  // Nothing — calibration is one-shot in setup()
}
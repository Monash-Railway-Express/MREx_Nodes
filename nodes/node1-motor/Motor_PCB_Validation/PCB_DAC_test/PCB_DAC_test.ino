/**
 * PCB DAC test file
 *
 * File:            PCB_DAC_test.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    13/03/2026
 * Last Modified:   13/03/2026
 * Version:         1.0.1
 *
 * Description:
 *   Tests the DAC outputs by generating independent ramp signals
 *   on VOUT0 and VOUT1.
 */

#include <Arduino.h>
#include <Wire.h>

#define DAC_ADDR 0x48
#define SDA_PIN  GPIO_NUM_2
#define SCL_PIN  GPIO_NUM_1

const int maxDAC = 1023;   // 10‑bit DAC
const int stepSize = 10;
const int stepDelay = 20;

int dacValue0 = 0;
int dacValue1 = 1023;

bool rampUp0 = true;
bool rampUp1 = false;

// ------------------------------------------------------------
// Configure DAC: enable internal reference, enable VOUT0/1
// ------------------------------------------------------------
void commonConfig() {
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x1F);   // COMMON-CONFIG register
  Wire.write(0x02);   // VOUT0 ON, IOUT0 OFF
  Wire.write(0x01);   // VOUT1 ON, IOUT1 OFF
  Wire.endTransmission();
}

// ------------------------------------------------------------
// Write 10‑bit DAC value to channel 0 or 1
// ------------------------------------------------------------
void writeDAC(uint8_t channel, uint16_t value) {
  if (value > 1023) value = 1023;

  uint16_t regVal = value << 6;  // left‑align 10‑bit into 16‑bit

  uint8_t reg = (channel == 0) ? 0x1C : 0x19; // DAC‑0‑DATA / DAC‑1‑DATA

  Wire.beginTransmission(DAC_ADDR);
  Wire.write(reg);
  Wire.write(regVal >> 8);
  Wire.write(regVal & 0xFF);
  Wire.endTransmission();
}

// ------------------------------------------------------------
// Update ramp value for a channel
// ------------------------------------------------------------
void updateRamp(int &value, bool &direction) {
  if (direction) {
    value += stepSize;
    if (value >= maxDAC) {
      value = maxDAC;
      direction = false;
    }
  } else {
    value -= stepSize;
    if (value <= 0) {
      value = 0;
      direction = true;
    }
  }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("DAC ramp test starting...");

  Wire.begin(SDA_PIN, SCL_PIN);

  commonConfig();
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------
void loop() {
  writeDAC(0, dacValue0);
  writeDAC(1, dacValue1);

  Serial.printf("DAC0: %d   DAC1: %d\n", dacValue0, dacValue1);

  updateRamp(dacValue0, rampUp0);
  updateRamp(dacValue1, rampUp1);

  delay(stepDelay);
}
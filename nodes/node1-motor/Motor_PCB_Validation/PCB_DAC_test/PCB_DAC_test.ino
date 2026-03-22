/**
 * PCB DAC test  file 
 *
 * File:            PCB_DAC_test.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    13/03/2026
 * Last Modified:   13/03/2026
 * Version:         1.0.0
 *
 */
//Tests the dac output 


#include <Arduino.h>
#include <Wire.h>

#define DAC_ADDR 0x48
#define SDA_PIN  GPIO_NUM_2
#define SCL_PIN  GPIO_NUM_1

const int maxDAC = 1023;   // 10-bit
const int stepSize = 10;
const int stepDelay = 20;

int dacValue0 = 1022;
int dacValue1 = 0;
bool rampUp = true;

void commonConfig() {
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x1F);   // COMMON-CONFIG register
  Wire.write(0x02);   // EN-INT-REF=0, VOUT0 ON, IOUT0 OFF
  Wire.write(0x01);   // VOUT1 ON, IOUT1 OFF
  Wire.endTransmission();
}

void writeDAC(uint8_t channel, uint16_t value) {
  if (value > 1023) value = 1023;
  uint16_t regVal = value << 6;                 // left-align 10-bit

  uint8_t reg = (channel == 0) ? 0x1C : 0x19;   // DAC-0-DATA / DAC-1-DATA

  Wire.beginTransmission(DAC_ADDR);
  Wire.write(reg);
  Wire.write(regVal >> 8);
  Wire.write(regVal & 0xFF);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("DAC ramp test starting...");

  Wire.begin(SDA_PIN, SCL_PIN);

  commonConfig();   // power up VOUT0/1, enable internal ref
}

void loop() {
  writeDAC(0, dacValue0);
  writeDAC(1, dacValue1);

  Serial.print("DAC Value0: ");
  Serial.println(dacValue0);
  Serial.print("DAC Value1: ");
  Serial.println(dacValue1);


  
  // if (rampUp) {
  //   dacValue0 += stepSize;
  //   if (dacValue0 >= maxDAC) {
  //     dacValue0 = maxDAC;
  //     rampUp = false;
  //   }
  // } else {
  //   dacValue0 -= stepSize;
  //   if (dacValue0 <= 0) {
  //     dacValue0 = 0;
  //     rampUp = true;
  //   }
  // }

  // if (rampUp) {
  //   dacValue1 += stepSize;
  //   if (dacValue1 >= maxDAC) {
  //     dacValue1 = maxDAC;
  //     rampUp = false;
  //   }
  // } else {
  //   dacValue1 -= stepSize;
  //   if (dacValue1 <= 0) {
  //     dacValue1 = 0;
  //     rampUp = true;
  //   }
  // }

  delay(stepDelay);
}
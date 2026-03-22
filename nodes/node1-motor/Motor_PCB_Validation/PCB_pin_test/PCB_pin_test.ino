/**
 * PCB pin test  file 
 *
 * File:            PCB_pin_test.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    13/03/2026
 * Last Modified:   13/03/2026
 * Version:         1.0.0
 *
 */
//Tests the level shifter output


#include <Arduino.h>

struct PinTest {
  const char* name;
  int gpio;
};

// All pins from your motor-node firmware
PinTest pins[] = {
  {"THROTTLE_SWITCH",        GPIO_NUM_4},
  {"REVERSING_SWITCH_MC",    GPIO_NUM_5},
  {"BRAKE_SWITCH",           GPIO_NUM_6},
  {"MOTOR1_GREEN_LED",       GPIO_NUM_7},
  {"MOTOR2_GREEN_LED",       GPIO_NUM_15},
  {"MOTOR1_RED_LED",         GPIO_NUM_16},
  {"MOTOR2_RED_LED",         GPIO_NUM_17},
  {"ENCODER_A",              GPIO_NUM_18},
  {"ENCODER_B",              GPIO_NUM_8},
  {"ENCODER_Z",              GPIO_NUM_9},
  {"REVERSING_CONTACTOR",    GPIO_NUM_10},
  {"EXTRA_GPIO_11",          GPIO_NUM_11},
  {"EXTRA_GPIO_12",          GPIO_NUM_12},
  {"EXTRA_GPIO_21",          GPIO_NUM_21},
  {"EXTRA_GPIO_47",                 GPIO_NUM_47},
  {"EXTRA_GPIO_48",                 GPIO_NUM_48}
};

const int numPins = sizeof(pins) / sizeof(pins[0]);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== MOTOR NODE GPIO TEST STARTED ===");
  Serial.println("One pin will be driven HIGH at a time.");
  Serial.println("Probe with multimeter to verify wiring.\n");

  // Set all pins to OUTPUT + LOW
  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i].gpio, OUTPUT);
    digitalWrite(pins[i].gpio, LOW);
  }
}

void loop() {

  for (int i = 0; i < numPins; i++) {

    // Turn all pins off
    for (int j = 0; j < numPins; j++) {
      digitalWrite(pins[j].gpio, LOW);
    }

    // Turn ONE pin on
    digitalWrite(pins[i].gpio, HIGH);

    Serial.print("ACTIVE: ");
    Serial.print(pins[i].name);
    Serial.print("  (GPIO ");
    Serial.print(pins[i].gpio);
    Serial.println(")");

    delay(4000);  // 1.5 seconds per pin
  }
}
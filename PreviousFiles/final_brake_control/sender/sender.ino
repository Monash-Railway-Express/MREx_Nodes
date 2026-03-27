#include "CAN_MREx.h"

// This is the "controller" node
const uint8_t nodeID = 1;

// Use valid CAN pins for your ESP32 + wiring
#define TX_GPIO_NUM GPIO_NUM_18
#define RX_GPIO_NUM GPIO_NUM_19

// Your toggle switch input pin (change to cirrect pin)
#define SWITCH_PIN  GPIO_NUM_21

// Track last switch state so we only send when it changes
uint8_t lastSwitch = 255;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Mode Controller started (Node 1)");

  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // Optional: start as operational (not required for sending NMT)
  nodeOperatingMode = 0x01;
}

void loop() {
  // Keep CAN stack serviced
  handleCAN(nodeID);

  // Read switch (INPUT_PULLUP => LOW means switch pressed/ON)
  uint8_t sw = (digitalRead(SWITCH_PIN) == LOW) ? 1 : 0;

  // Only act when switch changes
  if (sw != lastSwitch) {
    lastSwitch = sw;

    if (sw == 1) {
      // Switch ON => Operational
      Serial.println("Switch ON -> Sending NMT OPERATIONAL (0x01) to Node 2 (brakes)");
      sendNMT(0x01, 0x02);  // mode, target node (set the operating mode to 0x01 (operational mode) to node 0x02)
    } else {
      // Switch OFF => Pre-Operational (brakes apply)
      Serial.println("Switch OFF -> Sending NMT PRE-OP (0x80) to Node 2 (brakes)");
      sendNMT(0x80, 0x02); // (set the operating mode to 0x80 (pre op mode) to node 0x02)
    }
  }
}

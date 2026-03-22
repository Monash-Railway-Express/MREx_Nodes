#include "CAN_MREx.h"

const uint8_t nodeID = 2; // brakes node id is 2

#define TX_GPIO_NUM GPIO_NUM_18
#define RX_GPIO_NUM GPIO_NUM_19
#define SERVICE_BRAKE_PIN GPIO_NUM_22  // HIGH energises relay -> 24V to coil -> Release brakes
#define  LED_LIGHT GPIO_NUM_33
uint8_t lastOut = 255 ;// remembers what the brake output was last time
// manually set it high at the start because when the code runs it will se a change and print a message




void setup() {
  Serial.begin(9600);
  delay(1000); // wait to help connection
  Serial.println("Brakes node started (Node 2)");

  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);
  xTaskCreatePinnedToCore(
    CAN_Task,
    "CAN Task",
    4096,
    &nodeID,   // <--- passed into pvParameters
    3,
    NULL,
    0
  );

  pinMode(SERVICE_BRAKE_PIN, OUTPUT);
  pinMode(LED_LIGHT, OUTPUT);

  // Fail-safe startup: APPLY brakes in the beginning (no power to coil)
  digitalWrite(SERVICE_BRAKE_PIN, LOW);

  // IMPORTANT: Do NOT force operational mode here. ONLY for testing
  // nodeOperatingMode should be controlled by NMT messages on the CAN network.
  // nodeOperatingMode = 0x01; 
}

void loop() {
  // Always process CAN traffic (including NMT state changes)

  uint8_t out; // variable for brake pin output

  // STOPPED (0x02) or PRE-OP (0x80) => APPLY friction brakes => relay OFF
  if (nodeOperatingMode == 0x02 || nodeOperatingMode == 0x80) {
    out = LOW;   // relay de-energised -> no 24V -> brake APPLIED
    digitalWrite(LED_LIGHT,LOW);

  }
  // OPERATIONAL (0x01) => RELEASE friction brakes => relay ON
  else if (nodeOperatingMode == 0x01) {
    out = HIGH;  // relay energised -> 24V -> brake RELEASED
    digitalWrite(LED_LIGHT,HIGH);
  }
  // Unknown state => fail-safe APPLY
  else {
    out = LOW;
    digitalWrite(LED_LIGHT,HIGH);
  }

  // Only update + print when it changes, to make seeing changes more clear
  if (out != lastOut) {
    lastOut = out; // chance old state to new state
    digitalWrite(SERVICE_BRAKE_PIN, out); // write new state to brakes pin

    // printing status of braeks
    Serial.print("Mode = 0x");
    Serial.print(nodeOperatingMode, HEX);
    Serial.print(" | Friction brake: ");
    Serial.println(out == HIGH ? "RELEASED (relay ON)" : "APPLIED (relay OFF)");
  }
}
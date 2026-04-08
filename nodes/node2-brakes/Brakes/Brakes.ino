#include <CAN_MREx.h>
#include <Adafruit_NeoPixel.h>

uint8_t nodeID = 2; // brakes node id is 2

#define TX_GPIO_NUM GPIO_NUM_8 //18
#define RX_GPIO_NUM GPIO_NUM_7 //19
#define SERVICE_BRAKE_PIN GPIO_NUM_9  // 22 HIGH energises relay -> 24V to coil -> Release brakes
//LED pins

#define  LED_LIGHT GPIO_NUM_38
#define LED_COUNT 1
Adafruit_NeoPixel pixel(LED_COUNT, LED_LIGHT, NEO_GRB + NEO_KHZ800);

uint8_t lastOut = 255 ;// remembers what the brake output was last time
// manually set it high at the start because when the code runs it will se a change and print a message

// testing


void setup() {
  Serial.begin(9600);
  delay(1000); // wait to help connection
  Serial.println("Brakes node started (Node 2)");

  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  pinMode(SERVICE_BRAKE_PIN, OUTPUT);
  // pinMode(LED_LIGHT, OUTPUT);
  //onboard LED setup
  pixel.begin();
  pixel.clear();
  pixel.show();


  // Fail-safe startup: APPLY brakes in the beginning (no power to coil)
  digitalWrite(SERVICE_BRAKE_PIN, LOW);

  // IMPORTANT: Do NOT force operational mode here. ONLY for testing
  // nodeOperatingMode should be controlled by NMT messages on the CAN network.
  // nodeOperatingMode = 0x01; 

  xTaskCreatePinnedToCore(
      CAN_Task,
      "CAN Task",
      6144,
      &nodeID,
      3,
      NULL,
      0
  );


 
 


}



void loop() {
  // Always process CAN traffic (including NMT state changes)
  // handleCAN(nodeID);

  uint8_t out; // variable for brake pin output

  // STOPPED (0x02) or PRE-OP (0x80) => APPLY friction brakes => relay OFF
  if (nodeOperatingMode == 0x02 || nodeOperatingMode == 0x80) {
    out = LOW;   // relay de-energised -> no 24V -> brake APPLIED
    // digitalWrite(LED_LIGHT,LOW);
    pixel.setPixelColor(0, pixel.Color(50, 0, 0)); // red = applied
    pixel.show();

  }
  // OPERATIONAL (0x01) => RELEASE friction brakes => relay ON
  else if (nodeOperatingMode == 0x01) {
    out = HIGH;  // relay energised -> 24V -> brake RELEASED
    // digitalWrite(LED_LIGHT,HIGH);
    pixel.setPixelColor(0, pixel.Color(0, 50, 0)); // green = released
    pixel.show();

  }
  // Unknown state => fail-safe APPLY
  else {
    out = LOW;
    // digitalWrite(LED_LIGHT,HIGH);
    pixel.setPixelColor(0, pixel.Color(50, 0, 0)); // red = applied
    pixel.show();

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
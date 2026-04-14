/**
 * CAN MREX main (Template) file 
 *
 * File:            brakes_control_prototype.ino
 * Organisation:    MREX
 * Author:          Awon Girum & Fateh 
 * Date Created:    25/11/2025
 * Last Modified:   25/11/2025
 * Version:         1.11.0
 *
 */


#include "CM.h" // inlcudes all CAN MREX files

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 1;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_18 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_19 // Set GPIO pins for CAN Receive

// Brake control
const int brakeControlPin = 18;  // Relay

// Manual push button
const int buttonPin = 17;        // Push button input


// --- OD definitions ---


// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Serial Coms started at 9600 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---


  // --- Register TPDOs ---
  

  // --- Register RPDOs ---


  // --- Set pin modes ---
  pinMode(brakeControlPin, OUTPUT);

  // Inputs with pull-ups
  // pinMode(brakeOKPin, INPUT_PULLUP);
  // pinMode(brakeWornPin, INPUT_PULLUP);
  // pinMode(handReleaseNC, INPUT_PULLUP);
  // pinMode(handReleaseNO, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    handleCAN(nodeID);
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    handleCAN(nodeID);
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    handleCAN(nodeID);
  }

  //User code end loop() --------------------------------------------------------
}






// --- Pin Definitions ---
// Brake control
const int brakeControlPin = 18;  // Relay

// Manual push button
const int buttonPin = 17;        // Push button input

// State variables
// volatile bool brakeEngaged = false;
// volatile bool brakeOK = true;


// --- Interrupt Service Routines ---
// void IRAM_ATTR toggleBrake() {
//   brakeEngaged = !brakeEngaged;
// }

// void IRAM_ATTR updateBrakeOK() {
//   brakeOK = digitalRead(brakeOKPin) == LOW;
// }

// void IRAM_ATTR updateBrakeWorn() {
//   brakeWorn = digitalRead(brakeWornPin) == LOW;
// }

// void IRAM_ATTR updateHandNormal() {
//   handNormal = digitalRead(handReleaseNC) == LOW;
// }

// void IRAM_ATTR updateHandReleased() {
//   handReleased = digitalRead(handReleaseNO) == LOW;
}

void setup() {
  Serial.begin(9600);

  // Outputs
  pinMode(brakeControlPin, OUTPUT);

  // Inputs with pull-ups
  // pinMode(brakeOKPin, INPUT_PULLUP);
  // pinMode(brakeWornPin, INPUT_PULLUP);
  // pinMode(handReleaseNC, INPUT_PULLUP);
  // pinMode(handReleaseNO, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleBrake, FALLING);
  // attachInterrupt(digitalPinToInterrupt(brakeOKPin), updateBrakeOK, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(brakeWornPin), updateBrakeWorn, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(handReleaseNC), updateHandNormal, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(handReleaseNO), updateHandReleased, CHANGE);
}

void loop() {
  // --- Display brake condition ---
  // if (brakeOK && !brakeWorn) {
  //   Serial.println("Brake is OK.");
  // } else if (!brakeOK && brakeWorn) {
  //   Serial.println("Brake needs maintenance (wear or air gap issue).");
  // } else if (brakeOK && brakeWorn) {
  //   Serial.println("Intermediate wear detected.");
  // } else {
  //   Serial.println("Brake switch not activated or wiring issue.");
  }

  // --- Display hand-release status ---
  // if (handNormal && !handReleased) {
  //   Serial.println("Hand-release is NOT activated.");
  // } else if (!handNormal && handReleased) {
  //   Serial.println("Hand-release is ACTIVATED!");
  // } else {
  //   Serial.println("Hand-release switch in transitional or error state.");
  // }

  // // --- Apply brake state ---
  // if (brakeEngaged && handNormal && !brakeWorn) {
  //   digitalWrite(brakeControlPin, HIGH);  // Engage brake
  // } else {
  //   digitalWrite(brakeControlPin, LOW);   // Release brake
  }

  // --- Apply brake state ---
  if (brakeEngaged && handNormal && !brakeWorn) {
    digitalWrite(brakeControlPin, HIGH);  // Engage brake
  } else {
    digitalWrite(brakeControlPin, LOW);   // Release brake
  }

  delay(100);  // Small delay for readability
}

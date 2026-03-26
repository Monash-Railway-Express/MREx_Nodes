/**
 * Motor node ino file 
 *
 * File:            MotorVeroboard.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    1/03/2026
 * Last Modified:   22/03/2026
 * Version:         1.0.1
 *
 */


#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include <Arduino.h>

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
uint8_t nodeID = 1;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_4 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_5 // Set GPIO pins for CAN Receive
#define REGEN_BRAKE_PIN GPIO_NUM_13
#define MOTOR_PIN GPIO_NUM_14
#define REVERSING_PIN GPIO_NUM_10

// --- OD definitions ---
uint16_t od_motor_command = 0;
uint16_t regenBrake = 0;
uint8_t serviceBrake = 0;
uint8_t directionMode = 1;

//OPTIONAL: timing for a non blocking function occuring every two seconds
unsigned long previousMillis = 0;
const long interval = 100; // 100 milliseconds

// setting PWM properties
const int freq = 5000;
const int resolution = 8;

//Other variables
// Locks motor when both motor and brakes are applied and only releases when
// Both are 0
uint8_t motor_lockout = 1; 


// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
  //Initialize CANMREX protocol
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

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x60FF, 0x00, 2, sizeof(od_motor_command), &od_motor_command);
  registerODEntry(0x3012, 0x00, 2, sizeof(regenBrake), &regenBrake);
  registerODEntry(0x3012, 0x01, 2, sizeof(serviceBrake), &serviceBrake);
  registerODEntry(0x6060, 0x00, 2, sizeof(directionMode), &directionMode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
      {0x3012, 0x01, 8},  // Example: index 0x2000, subindex 1, 16 bits
    };
  mapTPDO(0, tpdoEntries, 1); //TPDO 1, entries, num entries

  // --- Register RPDOs ---
  configureRPDO(0, 0x180 + 3, 255, 0);         // COB-ID, transType, inhibit

  PdoMapEntry rpdoEntries[] = {
    {0x60FF, 0x00, 16},  // Example: index 0x2000, subindex 1, 16 bits
    {0x3012, 0x00, 16}    // Example: index 0x2001, subindex 0, 16 bits
  };
  mapRPDO(0, rpdoEntries, 2);

  // --- Set pin modes ---
  ledcAttach(MOTOR_PIN, freq, resolution);
  ledcAttach(REGEN_BRAKE_PIN, freq, resolution);
  pinMode(REVERSING_PIN, OUTPUT);


  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    ledcWrite(MOTOR_PIN, 0);
    ledcWrite(REGEN_BRAKE_PIN, 0);
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    ledcWrite(MOTOR_PIN, 0);
    ledcWrite(REGEN_BRAKE_PIN, 0);
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (od_motor_command > 10 && regenBrake > 10) {
    motor_lockout = 1;   // lock motor
    }
    else if (od_motor_command <= 10 && regenBrake <= 10) {
        motor_lockout = 0;   // unlock motor
    }

    // Declare variables outside the conditional blocks
    uint8_t motorpwmValue = 0;
    uint8_t brakepwmValue = 0;

    // Decision logic
    if(od_motor_command > 10 && regenBrake <= 10 && motor_lockout == 0){
      motorpwmValue = od_motor_command >> 2;
      brakepwmValue = 0;
      serviceBrake = 0;
    }
    else if(od_motor_command > 10 && regenBrake > 10){
      motorpwmValue = 0;
      brakepwmValue = regenBrake >> 2;
      serviceBrake = 0;
    }
    else if(od_motor_command <= 10 && regenBrake > 10){
      motorpwmValue = 0;
      brakepwmValue = regenBrake >> 2;
      serviceBrake = 1;
    }
    else{
      motorpwmValue = 0;
      brakepwmValue = 0;
      serviceBrake = 0;
    }

    // Get direction mode
    // uint32_t directionMode = executeSDORead(nodeID, 3, 0x6060, 0x00); // CHANGE THIS SO YOU CAN ONLY CHANGE IN PREOP
    if (directionMode == 1) {
      digitalWrite(REVERSING_PIN, LOW);
    } else if (directionMode == 3) {
      digitalWrite(REVERSING_PIN, HIGH);
    }
    
    // Apply PWM outputs
    ledcWrite(MOTOR_PIN, motorpwmValue);
    ledcWrite(REGEN_BRAKE_PIN, brakepwmValue);

    // Debug output
    Serial.print("Motor value: ");
    Serial.println(motorpwmValue);
    Serial.print("Brake value: ");
    Serial.println(brakepwmValue);
    Serial.print("Service brake: ");
    Serial.println(serviceBrake);
    Serial.print("Direction mode: ");
    Serial.println(directionMode);
  }
}

  //User code end loop() --------------------------------------------------------
}
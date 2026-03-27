  /**
 * CAN MREX main (Template) file 
 *
 * File:            Brake Control.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    5/08/2025
 * Last Modified:   1/10/2025
 * Version:         1.11.0
 *
 */


#include "CM.h" // inlcudes all CAN MREX files
#include <Arduino.h>
#include "esp_task_wdt.h"  // Include ESP32 watchdog library

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 1;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_18 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_19 // Set GPIO pins for CAN Receive

const int RELAY_PIN = 22;
const int LED_PIN = 32; // LED indicator
const int BRAKE1_SENSOR_PIN = 26; // Current sensor for Brake 1
const int BRAKE2_SENSOR_PIN = 27; // Current sensor for Brake 2
//const int CAN_TX_PIN = 18; // CAN TX
//const int CAN_RX_PIN = 19; // CAN RX

// --- OD definitions ---
uint32_t brakestate = 0

// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Serial Coms started at 9600 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x1000, 0x01, 2, sizeof(brakestate), &brakestate) // Index, Subindex, Read Write access, Size, Data

  // --- Register TPDOs ---
  //configureTPDO(0, 0x180 + nodeID, 255, 10, 10) // COB-ID, transType, inhibit, event

  // --- Register RPDOs ---
  configureRPDO(0, 0x180 + nodeID, 255, 0) // COB-ID, transType, inhibit
  
  PdoMapEntry rpdoEntries[] = {
    {0x2000, 0x01, 16}
    {0x2001, 0x00, 8}
  };
  mapRPDO(0, rpdoEntries, 2)
  // --- Set pin modes ---
 

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
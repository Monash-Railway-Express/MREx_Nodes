/**
 * CAN MREX RPDOs file 
 *
 * File:            RPDOs.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    1/10/2025
 * Last Modified:   20/10/2025
 * Version:         1.11.0
 *
 */

#include "CAN_MREx.h" // inlcudes all CAN MREX files

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 2;  // Change this to set your device's node ID. I mean 2 is fine for testing.

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // Set GPIO pins for CAN Receive

// --- OD definitions ---
uint32_t current_can = 0;
uint16_t voltage_can = 0;
uint32_t power_can = 0;
uint16_t soc_can = 0;
int32_t recovered_energy_can = 0;

const unsigned long interval = 1000;
unsigned long  previousMillis = 0;

float current = 0;
float voltage = 0;
float soc = 0;
int32_t power = 0;
uint32_t recovered_energy = 0;

// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x2001, 0x00, 0, sizeof(current_can), &current_can);
  registerODEntry(0x2001, 0x01, 0, sizeof(voltage_can), &voltage_can); 
  registerODEntry(0x2001, 0x02, 0, sizeof(soc_can), &soc_can); 
  registerODEntry(0x2001, 0x03, 0, sizeof(power_can), &power_can); 
  registerODEntry(0x2001, 0x04, 0, sizeof(recovered_energy_can), &recovered_energy_can); 

  // --- Register TPDOs ---
  // this node not transmitting anything only receiving
  

  // --- Register RPDOs ---
  configureRPDO(0, 0x180 + 7, 255, 500);         // COB-ID, transType, inhibit
  configureRPDO(1, 0x280 + 7, 255, 500);         // COB-ID, transType, inhibit

  PdoMapEntry rpdoEntries1[] = {
    {0x2001, 0x00, 32}, 
    {0x2001, 0x01, 16},    
    {0x2001, 0x02, 16}
  };

  PdoMapEntry rpdoEntries2[] = {
    {0x2001, 0x03, 32},  
    {0x2001, 0x04, 32}    
  };

  mapRPDO(0, rpdoEntries1, 3);
  mapRPDO(1, rpdoEntries2, 2);

  nodeOperatingMode = 0x01;

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
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      current = ((int32_t)current_can - 1000000)/1000.0; //subtract offset and divide by 1000 to get correct current in A
      voltage = (voltage_can)/1000.0; // get voltage in V
      soc = (soc_can)/10.0;
      power = ((int32_t)power_can - 1000000); // subtract offset to get correct power in W. Power is already in W so no need to divide.
      recovered_energy = recovered_energy_can;

      Serial.print("Current: ");
      Serial.println(current,3);
      Serial.print("Voltage: ");
      Serial.println(voltage,3);
      Serial.print("State of Charge: ");
      Serial.println(soc,1); 
      Serial.print("Power: ");
      Serial.println(power);
      Serial.print("Recovered Energy: ");
      Serial.println(recovered_energy);
    }
  }

  //User code end loop() --------------------------------------------------------
}

/**
 * @file battery.ino
 * @brief Code to compute recovered energy
 * @details The code computes recovered energy using the power values received from the smart shunt. It also transmits current, voltage, SOC, power , and recovered energy onto the can bus.
 * It uses VedirectFrameHandler.cpp and VeDirectFrameHandler.h files which parse data received from the shunt and stores in a buffer that can be read.
 * @author Hoor E Jannat Urboshi
 * @author Arjuna Edirisinghe
 * @date 		17/04/2026
 * @version 2.0.0
 * @organisation MREX
 * @see VeDirectFrameHandler.c and VeDirectFrameHandler.h
 */

#include <Arduino.h>
#include <stdint.h>
#include <CAN_MREx.h>
#include <HardwareSerial.h>

#include "VeDirectFrameHandler.h" // ve direct parser

// =============================================================================
// Constants and other
// =============================================================================

//Defining Node ID's
#define MOTOR_ID 0x01
#define BRAKES_ID 0x02
#define DRIVER_ID 0x03
#define LIGHTS_ID 0x04
#define AUDIO_ID 0x05
#define AUTOSTOP_ID 0x06
#define BATTERY_ID 0x07
#define LOGGER_ID 0x08
#define LCD_ID 0x09

// --- Node ID ---
uint8_t NODE_ID = 0x07;

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // GPIO pins for CAN Receive

// --- Operating Modes ---
enum OperatingMode : uint8_t {
    MODE_STOPPED        = 0x02,
    MODE_PREOP          = 0x80,
    MODE_OPERATIONAL    = 0x01
};

// =============================================================================
// OD Variable Definitions
// =============================================================================

uint32_t current_can = 0;
uint16_t voltage = 0; // voltage always positive 
uint32_t power_can = 0; // Instantenous Power
uint16_t state_of_charge = 0; // 0-100%. +/- 0.1%. If the SOC is 88.3% it is sent as 883 so 16 bits enough.
uint32_t recovered_energy_can = 0;
uint16_t regen_brake = 0;
uint16_t motor_command = 0;


// =============================================================================
// Global Variables
// =============================================================================
int32_t current = 0; 
int32_t power = 0;
int32_t recovered_energy = 0;
uint32_t prev_power_sample = 0;
uint32_t new_power_sample = 0;
uint32_t power_sample = 0;
int32_t slice_area = 0;

bool prev_sample_1 = false; // flag that is set to true after the first power sample after regen starts is received
bool prev_sample_2 = false; // flag that is set to true after the first power sample after loco starts up again is received4
bool regen_occured = false;
unsigned long currentMillis = 0;
unsigned long last_data_received_time = 0;
const unsigned long shunt_data_interval = 1500; // If 1.5s pass without receiving data from the shunt, then send error message

VeDirectFrameHandler veParser;

HardwareSerial veSerial(2); // Use UART2 for shunt data

// User code end ---------------------------------------------------------

void setup() {

  Serial.begin(115200); 
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");

  veSerial.begin(19200, SERIAL_8N1, 16, 17); // GPIO 16 - RX; GPIO 17 -TX. // ESP32 <-> Shunt
  delay(1000);
  Serial.println("Reading values from shunt started at 19200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

  xTaskCreatePinnedToCore(
      CAN_Task,
      "CAN Task",
      6144,
      &NODE_ID,
      3,
      NULL,
      0
  );

  // --- Register OD entries ---
  registerODEntry(0x2000, 0x00, 2, sizeof(current_can), &current_can); 
  registerODEntry(0x2000, 0x01, 2, sizeof(voltage), &voltage); 
  registerODEntry(0x2000, 0x02, 2, sizeof(state_of_charge), &state_of_charge); 
  registerODEntry(0x2000, 0x03, 2, sizeof(power_can), &power_can);
  registerODEntry(0x2000, 0x04, 2, sizeof(recovered_energy_can), &recovered_energy_can);
  registerODEntry(0x606A, 0x00, 2, sizeof(motor_command), &motor_command);
  registerODEntry(0x3012, 0x00, 2, sizeof(regen_brake), &regen_brake);

  // --- Configure TPDOs and RPDOs ---

  configureTPDO(0, 0x180 + NODE_ID, 255, 100, 1000);  // TPDO 1, COB-ID, transType, inhibit, timer
  configureTPDO(1, 0x280 + NODE_ID, 255, 100, 1000);  // TPDO 2, COB-ID, transType, inhibit, timer
  configureRPDO(0, 0x180 + DRIVER_ID, 255, 0); // RPDO 1, COB-ID, transType, inhibit. This node receives from node 3 which is driver controls.

  // --- TPDO and RPDO entries ---
  
  PdoMapEntry tpdoEntries1[] = {
      {0x2000, 0x00, 32},    // current
      {0x2000, 0x01, 16},   // voltage
      {0x2000, 0x02, 16},   // state of charge   
    };

  PdoMapEntry tpdoEntries2[] = {   
      {0x2000, 0x03, 32},   // power
      {0x2000, 0x04, 32},   // recovered energy
    };
    
  PdoMapEntry rpdoEntries[] = {
    {0x606A, 0x00, 16},     // motor command
    {0x3012, 0x00, 16}      // regen
  };

  // --- Map TPDOs and RPDOs ---

  mapTPDO(0, tpdoEntries1, 3); //TPDO 1, entries, num entries
  mapTPDO(1, tpdoEntries2, 2); //TPDO 2, entries, num entries
  mapRPDO(0, rpdoEntries, 2); // RPDO 1, entries, num entries

}

/**
 * @brief reads the bytes from the shunt and passes them into rxData which puts the name-value pairs in the buffer veData
 * @return nothing
 */

void ReadVEData() {
    while (veSerial.available()){
        veParser.rxData(veSerial.read()); 
    }
    yield(); 
}

/**
 * @brief prints values including current, voltage, SOC, instantenous power AND recovered energy to serial monitor every second
 * @return nothing
 */

void EverySecond() {
    static unsigned long prev_millis;
    if (millis() - prev_millis > 1000) {
        PrintData();
        Serial.print("Recovered Energy");
        Serial.print(" = ");
        Serial.println(recovered_energy);
        prev_millis = millis();
    }
}

/**
 * @brief prints values in the buffer including current, voltage, SOC, and instantenous power to serial monitor every second
 * @return nothing
 */
void PrintData() {
    for ( int i = 0; i < veParser.veEnd; i++ ) {
    Serial.print(veParser.veData[i].veName);
    Serial.print(" = ");
    Serial.println(veParser.veData[i].veValue);    
    }
}

/**
 * @brief computes recovered energy
 * @return nothing
 */

void findRecoveredEnergy() {
  if (motor_command > 0 && regen_occured == true) { // If regen already occured and there is throttle that means we are restarting in the energy storage challenge so recovered energy should now decrement
  // if power is less than or equal or to 0 then don't decrement. Power should be positive because motor_command > 10 means we have throttle?
      if (power <= 0) {
      prev_sample_2 = false;
      recovered_energy_can = (uint32_t)recovered_energy;
      return;
      }
      // code below runs if power is positive
      prev_sample_1 = false; // This flag indicates if a power sample is the first power sample when regenerative braking is active. Necessary to reset in case we have regen for a second time.
      power_sample = (uint32_t)power; // Take absolute value of power. Explicit casting recommended.
      if (prev_sample_2 == false) {  // If prev_sample_2 is false that means this power sample is the first sample we have when power is positive or it is the first power sample after we restarted
      prev_power_sample = power_sample; // save it as previous sample so that we can use it to calculate slice area
      prev_sample_2 = true; // so the next power samples are correctly NOT interpreted as the first power sample
      recovered_energy_can = (uint32_t)recovered_energy; // The recovered energy value will stay constant until we have a second sample. Two samples are needed to calculate slice area.
      return; // return because we cannot calculate area witj just one sample
      }
      new_power_sample = power_sample; 
      slice_area = ((prev_power_sample + new_power_sample)/2)*1; // calculate area of trapezium. The time between samples is 1 second so height of trapezium is 1.
      if (slice_area >= recovered_energy) { // If the energy we need to subract is greater than the recovered energy value clamp it to 0 so it never goes negative
        recovered_energy = 0;
      }
      else {
        recovered_energy -= slice_area; // Otherwise decrement the recovered energy
      }
      prev_power_sample = new_power_sample; // current sample will be previous sample for the next decrement calculation
      recovered_energy_can = (uint32_t)recovered_energy; // update remaining recovered energy value. Explicit casting is recommended.

  } else if (power < 0) { // If the power is negative that means we are in regen mode
      regen_occured = true; // set flag to indicate regen occured. This flag prevents us from decrementing the recovered energy when power is positive. We will only decrement for positive power after regen has occured which is desired
      prev_sample_2 = false; // reset in case we want to decrement for the second time after regeb has occurred for the second time
      power_sample = (uint32_t)-power; // take absolute value for easier calculation. Explicit casting is recommened.
      if (prev_sample_1 == false){ // this means this power value is the first power value when regenerative braking is in action
      prev_power_sample = power_sample; // make this sample previous sample. a second sample is required to calculate slice area
      prev_sample_1 = true; // flag set to true here so for the second sample onwards, code in the else condition will be executed. samples starting from the second sample are interpreted correctly as not the first sample.
      recovered_energy_can = (uint32_t)recovered_energy; // keep recovered energy value constant until we have a second value.
      return; // return because one sample is not enough to calculate the area of the trapezoidal slide
      }
      new_power_sample = power_sample; 
      slice_area = ((prev_power_sample + new_power_sample)/2)*1;  // calculate area of trapezium. The time between samples is 1 second so height of trapezium is 1.
      recovered_energy += slice_area; // aggregate area of the trapeziums. increment recovered energy.
      prev_power_sample = new_power_sample;  // current sample will be previous sample for the next decrement calculation
      recovered_energy_can = (uint32_t)recovered_energy; // update remaining recovered energy value. Explicit casting is recommended.
  // Otherwise hold the value. In stopped state, hold the recovered energy value constant.
  } else {
      prev_sample_1 = false;
      prev_sample_2 = false;
      recovered_energy_can = (uint32_t)recovered_energy;
  }
}

/**
 * @brief updates OD entries. OD entries are updated every second when new data is received into the buffer
 * @return nothing
 */

void updateODentries(){
    // Iterate through all name-value pairs in the buffer. Find voltage, current, SOC, power in the buffer and assign the values to the variables in the object dictionary of the node.
    for (int i = 0; i < veParser.veEnd; ++i) {
       if (strcmp(veParser.veData[i].veName, "V") == 0){
        voltage = atoi(veParser.veData[i].veValue);
       }
       else if (strcmp(veParser.veData[i].veName, "I") == 0){
        current = atoi(veParser.veData[i].veValue);
        current_can = current; 
       }
       else if (strcmp(veParser.veData[i].veName, "SOC") == 0){
        state_of_charge = atoi(veParser.veData[i].veValue); // needs to be divided by 10 to get it in percentage. 995/10 = 99.5%.
       }
       else if (strcmp(veParser.veData[i].veName, "P") == 0){
        power = atoi(veParser.veData[i].veValue);
        power_can = power; 
        findRecoveredEnergy(); 
       }
  }
  veParser.clearData();
}


void loop(){

  ReadVEData();  // read data from shunt and put into buffer
  EverySecond(); // Debug: print the data contained in the buffer every second.

  switch (nodeOperatingMode) {

    // In stopped mode, do nothing
    case MODE_STOPPED: break;

    // In pre-op mode, we check if the data is received from the shunt, parsed and stored in the buffer 
    case MODE_PREOP:
      if (veParser.isDataAvailable()) {
        updateODentries();  
        last_data_received_time = millis(); // if new data is received update the OD entries and record the time
      } else {
        // if new data is not received
        currentMillis = millis();
        if (currentMillis - last_data_received_time >= shunt_data_interval) { // if more than 1.5s pass and still no data in buffer then raise error because a new block should be received every sec.
          // sendEMCY(1,nodeID, 0x00000701); // Minor Emergency
          Serial.println("No Data in the buffer!");
          last_data_received_time = currentMillis;
        }
      }
      break;

    // in operational mode, 
    case MODE_OPERATIONAL:
      if (nodeOperatingMode == 0x01){ 
        if (veParser.isDataAvailable()) { // Update OD entries every s. A new block is received every second.
          updateODentries();   
        }
      }
      break;

    default: break;
  }

  /**
  Deprecated
  */
  // // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  
  // if (nodeOperatingMode == 0x80){ 
  //   ReadVEData(); // this function passes each incoming byte into rxData. rxData stores the name-value pairs in the buffer (array of structs - 1 struct is 1 name-value pair).
  //   // EverySecond(); // Debug: print the data in the buffer every second.
  //   if (veParser.isDataAvailable()) {
  //     updateODentries();  
  //     last_data_received_time = millis(); // if new data is received update the OD entries and record the time
  //   } else {
  //     // if new data is not received
  //   currentMillis = millis();
  //   if (currentMillis - last_data_received_time >= shunt_data_interval) { // if more than 1.5s pass and still no data in buffer then raise error because a new block should be received every sec.
  //     sendEMCY(1,nodeID, 0x00000701); // Minor Emergency
  //     Serial.println("No Data in the buffer!");
  //     last_data_received_time = currentMillis;
  //     }
  //   }
  // }

  // // --- Operational state (Normal operating mode) ---
  // if (nodeOperatingMode == 0x01){ 
  //   ReadVEData(); // this function passes each incoming byte into rxData. rxData stores the name-value pairs in the buffer (array of structs - 1 struct is 1 name-value pair).
  //   // EverySecond();
  //   if (veParser.isDataAvailable()) { // Update OD entries every s. A new block is received every second.
  //       updateODentries();   
  //     }
  //   }
}






























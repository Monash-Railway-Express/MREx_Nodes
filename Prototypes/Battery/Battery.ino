/**
 * CAN MREX main (Template) file 
 *
 * File:            main.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    5/08/2025
 * Last Modified:   1/10/2025
 * Version:         1.11.0
 *
 */
/*#include "CAN_MREx.h" // inlcudes all CAN MREX files
#include "VeDirectFrameHandler.h"
#include "Arduino.h"
#include <HardwareSerial.h>

// User code begin: ------------------------------------------------------
VeDirectFrameHandler myparser;

// --- CAN MREx initialisation ---
const uint8_t nodeID = 7;  // battery is node 7
const bool nmtMaster = false;
const bool heartbeatConsumer = false;

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // GPIO pins for CAN Receive
HardwareSerial veSerial(2); // Use UART2 for sensor data

// --- OD definitions ---
uint8_t current_sign = 0; // 1 byte to indicate sign of current.
uint32_t current_magnitude = 0;
uint16_t voltage = 0; // voltage always positive DC
uint8_t power_sign = 0; 
uint16_t power_magnitude = 0; // Instantenous Power
uint16_t state_of_charge = 0; // 0-100%. +/- 0.1%. If the SOC is 88.3% it is sent as 883 so 16 bits enough.
uint8_t Ah_sign = 0; // 1 byte to indicate sign of Ah consumption. 
uint32_t Amp_hours_consumed_magnitude = 0; 
uint32_t recovered_energy_can = 0;

// variables not for OD
int32_t ce = 0;
int16_t power = 0;
int32_t current = 0;
int32_t recovered_energy = 0; // how much energy was recovered from regenerative braking
uint16_t prev_power_sample = 0;
uint16_t new_power_sample = 0;
uint16_t power_sample = 0;
uint16_t slice_area = 0;
//bool regen_mode = false;
bool prev_sample_1 = false;
bool prev_sample_2 = false;
unsigned long previousMillis = 0;
unsigned long last_data_received_time = 0;
const long interval = 1000; // 1s
const long sensor_data_interval = 1500; //1.5s
unsigned long time_regen_brake_start = 0;
unsigned long time_regen_brake_end = 0;
unsigned int regen_brake_duration = 0;

// User code end ---------------------------------------------------------

void setup() {
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  Serial.begin(115200); 
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");

  veSerial.begin(19200, SERIAL_8N1, 16, 17); // GPIO 16 - RX; GPIO 17 -TX. // ESP32 <-> Shunt
  delay(1000);
  Serial.println("Reading values from shunt started at 19200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------

  // --- Register OD entries ---
  // values can only be read
  registerODEntry(0x2000, 0x00, 0, sizeof(current_sign), &current_sign); // Index, Subindex, Read Write access, Size, Data
  registerODEntry(0x2000, 0x01, 0, sizeof(current_magnitude), &current_magnitude); 
  registerODEntry(0x2000, 0x02, 0, sizeof(voltage), &voltage); 
  registerODEntry(0x2000, 0x03, 0, sizeof(Ah_sign), &Ah_sign); 
  registerODEntry(0x2000, 0x04, 0, sizeof(Amp_hours_consumed_magnitude), &Amp_hours_consumed_magnitude); 
  registerODEntry(0x2000, 0x05, 0, sizeof(state_of_charge), &state_of_charge); 
  registerODEntry(0x2000, 0x06, 0, sizeof(power_sign), &power_sign); 
  registerODEntry(0x2000, 0x07, 0, sizeof(power_magnitude), &power_magnitude);
  registerODEntry(0x2000, 0x08, 0, sizeof(recovered_energy_can), &recovered_energy_can);


  configureTPDO(0, 0x180 + nodeID, 255, 100, 1000);  // TPDO 1, COB-ID, transType, inhibit, event
  configureTPDO(1, 0x280 + nodeID, 255, 100, 1000);  // TPDO 2, COB-ID, transType, inhibit, event
  configureTPDO(2, 0x380 + nodeID, 255, 100, 1000);  // TPDO 3, COB-ID, transType, inhibit, event
  

  PdoMapEntry tpdoEntries1[] = {
      {0x2000, 0x00, 8},    
      {0x2000, 0x01, 32},   
      {0x2000, 0x02, 16},      
    };

  PdoMapEntry tpdoEntries2[] = {   
      {0x2000, 0x03, 8}, 
      {0x2000, 0x04, 32},    
      {0x2000, 0x05, 16},   
    };

  PdoMapEntry tpdoEntries3[] = {   
      {0x2000, 0x06, 8}, 
      {0x2000, 0x07, 16},
      {0x2000, 0x09, 32},
    };

    mapTPDO(0, tpdoEntries1, 3); //TPDO 1, entries, num entries
    mapTPDO(1, tpdoEntries2, 3); //TPDO 2, entries, num entries
    mapTPDO(2, tpdoEntries3, 3); //TPDO 3, entries, num entries

  // --- Register RPDOs ---
  // This node simply puts the sensor values on the can bus. Only TPDOs need to be configured
  // RPDOS need to be configured in node 3 to receive this data.
  
  // --- Set pin modes ---

  // User code Setup end ------------------------------------------------------

}

// This function reads the bytes from the shunt and passes them into rxData which puts the name-value pairs in the buffer veData
void ReadVEData() {
    while (veSerial.available()){
        myparser.rxData(veSerial.read()); 
    }
    yield(); 
}

// Print values to the serial monitor every second - a new block is received every second
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
// Print values to the serial monitor every second
void PrintData() {
    for ( int i = 0; i < myparser.veEnd; i++ ) {
    Serial.print(myparser.veData[i].veName);
    Serial.print(" = ");
    Serial.println(myparser.veData[i].veValue);    
    }
}
*/
/*
Okay let's start from the beginning. Locomotive turns on. Power is positive and recovered energy is 0 initially and it should stay as 0 (lines 193-194)
Now when regen starts, power should be negative then line 182 onwards is executed. The recovered energy should go up.
Then we stop and start the loco again, it should now move using the recovered energy so the recovered energy should go down. 
When we turn on the loco again, power is positive so we go to line 196, recovered energy should be some non zero positive value so line 200 onwards executed. We start decrementing that value.

Now, questions remains, under what condition would recovered energy be negative? Because initially it's 0. If power is positive and it is initially 0 then we keep it as 0, and if non-zero and positive then we decrement it? 
If the power is negative then it is incremented.
I don't know why the recovered energy would be negative. If we are decrementing, then there could be a possibility that it goes below 0 
but that doesn't make physical sense like if there is no recovered energy what are we even using?

Although recovered energy was unsigned before, the code was working fine. Was that because recovered energy never went to negative so it didn't matter if it was signed or unsigned?
With the motors, we get a crazy number because recovered energy was declared as unsigned so a negative number was not handled properly (wrapped around)? But why would it be negative.

Like Arjuna said, we can actually have "if (recovered_energy = 0)" instead of (recovered_energy <= 0) then recovered energy can be unsigned and can be sent through CAN bus. 
But based on what happened today that is overflow is because I am guessing recovered energy went to negative. So what I can do is have a different unsigned variable carry recovered energy over CAN Bus.

Wait no no no. There was no recovered energy today so why wasn't it just 0??? if there is no regen then recovered energy should remain as 0? 

*/
/*
void findRecoveredEnergy(){
if(power < 0){
  power_sample = -power; // take absolute value. Sign needs to be sent separately.
  if (prev_sample_1 == false){ // this means this power value is the first power value in regenerative braking mode
  prev_power_sample = power_sample;
  prev_sample_1 = true; // flag set to true here so for the second sample onwards, line 173 onwards should be executed
  return;
  }
  else {
    new_power_sample = power_sample; 
  }
  slice_area = ((prev_power_sample + new_power_sample)/2)*1; // the height, that is, interval between samples is 1 s. area of trapezium.
  recovered_energy += slice_area; // aggregate area of the trapeziums
  prev_power_sample = new_power_sample;
}
// if power not negative then we stop integrating
else {
  if (recovered_energy <= 0){
    recovered_energy = 0;
  } else {
  power_sample = power;
  if (prev_sample_2 == false){
  prev_power_sample = power_sample;
  prev_sample_2 = true;
  return;
  }
  else {
    new_power_sample = power_sample; 
  }
  slice_area = ((prev_power_sample + new_power_sample)/2)*1;
  recovered_energy -= slice_area;
  prev_power_sample = new_power_sample;
  }
}
  recovered_energy_can = recovered_energy;
}


void updateODentries(){
    // iterate through all name-value pairs in the buffer. Find voltage, current, SOC, and Ah in the buffer and assign the values to the variables in the object dictionary of the node.
    // find voltage, current, etc and update the OD variables with the values.
    for (int i = 0; i < myparser.veEnd; ++i) {
       if (strcmp(myparser.veData[i].veName, "V") == 0){
        voltage = atoi(myparser.veData[i].veValue);
       }
       else if (strcmp(myparser.veData[i].veName, "I") == 0){
        current = atoi(myparser.veData[i].veValue);
        if(current < 0){
        current_sign = 1; // negative current ; current sign byte 0000 0001
        current_magnitude = -current; // Absolute value of current in mA. Needs to be divided by 1000 to get current in A. 
        } else {
        current_sign = 0; // positive current ; current sign byte 0000 0000
        current_magnitude = current; 
        }
       }
       else if (strcmp(myparser.veData[i].veName, "SOC") == 0){
        state_of_charge = atoi(myparser.veData[i].veValue);     // needs to be divided by 10 to get it in percentage. 995/10 = 99.5%.
       }
       else if (strcmp(myparser.veData[i].veName, "CE") == 0){
        ce = atoi(myparser.veData[i].veValue); 
        if (ce < 0){
        Ah_sign = 1;  // negative amp-hour consumption
        Amp_hours_consumed_magnitude = -ce; // Needs to be divided by 1000 to get value in Ah.
        } else {
        Ah_sign = 0; 
        Amp_hours_consumed_magnitude = ce; 
        }
       }
       else if (strcmp(myparser.veData[i].veName, "P") == 0){
        power = atoi(myparser.veData[i].veValue);
        if(power < 0){
        power_sign = 1; // negative power
        power_magnitude = -power; // Value in watts W.
        } else {
        power_sign = 0; 
        power_magnitude = power; 
        }
        findRecoveredEnergy(); 
       }
  }
  myparser.clearData();
}

void loop(){
  //User Code begin loop() ----------------------------------------------------
  nodeOperatingMode = 0x80;
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    handleCAN(nodeID); // heartbeat is handled in handleCAN.
  }
  
  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    handleCAN(nodeID);
    // In pre-op state, check if data is being read from the shunt, parsed, and stored correctly in the buffer 
    ReadVEData(); // this function passes each incoming byte into rxData. rxData stores the name-value pairs in the buffer (array of structs - 1 struct is 1 name-value pair).
    EverySecond(); // Debug: print the data in the buffer every second.
    if (myparser.isDataAvailable()) {
      updateODentries();  
      last_data_received_time = millis();
    } else {
    unsigned long currentMillis = millis();
    if (currentMillis - last_data_received_time >= sensor_data_interval) { // if more than 1.5s pass and still no data in buffer then raise error because a new block should be received every sec.
      sendEMCY(1,nodeID, 0x00000301); 
      Serial.println("No Data in the buffer!");
      last_data_received_time = currentMillis;
      }
    }
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    handleCAN(nodeID);
    ReadVEData(); // this function passes each incoming byte into rxData. rxData stores the name-value pairs in the buffer (array of structs - 1 struct is 1 name-value pair).
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
    if (myparser.isDataAvailable()) { // Update OD entries every s. A new block is received every second.
        updateODentries();   
      }
    }
  }
  //User code end loop() --------------------------------------------------------
}
*/

#include "CAN_MREx.h" // inlcudes all CAN MREX files
#include "VeDirectFrameHandler.h" // parser
#include "Arduino.h"
#include <HardwareSerial.h>

// User code begin: ------------------------------------------------------
VeDirectFrameHandler myparser;

// --- CAN MREx initialisation ---
const uint8_t nodeID = 7;  // battery is node 7
const bool nmtMaster = false;
const bool heartbeatConsumer = false;

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // GPIO pins for CAN Receive
HardwareSerial veSerial(2); // Use UART2 for shunt data

// --- OD definitions ---
uint32_t current_can = 0;
uint16_t voltage = 0; // voltage always positive 
uint32_t power_can = 0; // Instantenous Power
uint16_t state_of_charge = 0; // 0-100%. +/- 0.1%. If the SOC is 88.3% it is sent as 883 so 16 bits enough.
uint32_t recovered_energy_can = 0;

// variables not for OD
int32_t current = 0; 
int32_t power = 0;
int32_t recovered_energy = 0;
uint32_t prev_power_sample = 0;
uint32_t new_power_sample = 0;
uint32_t power_sample = 0;
uint32_t slice_area = 0;

bool prev_sample_1 = false; // flag that is set to true after the first power sample after regen starts is received
bool prev_sample_2 = false; // flag that is set to true after the first power sample after loco starts up again is received
unsigned long currentMillis = 0;
unsigned long last_data_received_time = 0;
const unsigned long shunt_data_interval = 1500; // If 1.5s pass without receiving data from the shunt, then send error message

// User code end ---------------------------------------------------------

void setup() {

  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  Serial.begin(115200); 
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");

  veSerial.begin(19200, SERIAL_8N1, 16, 17); // GPIO 16 - RX; GPIO 17 -TX. // ESP32 <-> Shunt
  delay(1000);
  Serial.println("Reading values from shunt started at 19200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------

  // --- Register OD entries ---
 
  registerODEntry(0x2000, 0x00, 0, sizeof(current_can), &current_can); 
  registerODEntry(0x2000, 0x01, 0, sizeof(voltage), &voltage); 
  registerODEntry(0x2000, 0x02, 0, sizeof(state_of_charge), &state_of_charge); 
  registerODEntry(0x2000, 0x03, 0, sizeof(power_can), &power_can);
  registerODEntry(0x2000, 0x04, 0, sizeof(recovered_energy_can), &recovered_energy_can);

  configureTPDO(0, 0x180 + nodeID, 255, 100, 1000);  // TPDO 1, COB-ID, transType, inhibit, event
  configureTPDO(1, 0x280 + nodeID, 255, 100, 1000);  // TPDO 2, COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries1[] = {
      {0x2000, 0x00, 32},    
      {0x2000, 0x01, 16},   
      {0x2000, 0x02, 16},      
    };

  PdoMapEntry tpdoEntries2[] = {   
      {0x2000, 0x03, 32}, 
      {0x2000, 0x04, 32},     
    };

    mapTPDO(0, tpdoEntries1, 3); //TPDO 1, entries, num entries
    mapTPDO(1, tpdoEntries2, 2); //TPDO 2, entries, num entries

  // --- Register RPDOs ---
  // This node simply puts values from shunt on the can bus. Only TPDOs need to be configured
  // RPDOS need to be configured in node 3 to receive this data.
  
  // --- Set pin modes ---

  // User code Setup end ------------------------------------------------------

}

// This function reads the bytes from the shunt and passes them into rxData which puts the name-value pairs in the buffer veData
void ReadVEData() {
    while (veSerial.available()){
        myparser.rxData(veSerial.read()); 
    }
    yield(); 
}

// Print values to the serial monitor every second - a new block is received every second
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
// Print values to the serial monitor every second
void PrintData() {
    for ( int i = 0; i < myparser.veEnd; i++ ) {
    Serial.print(myparser.veData[i].veName);
    Serial.print(" = ");
    Serial.println(myparser.veData[i].veValue);    
    }
}

void findRecoveredEnergy(){
if(power < 0){
  power_sample = -power; // take absolute value. 
  if (prev_sample_1 == false){ // this means this power value is the first power value when regenerative braking is in action
  prev_power_sample = power_sample;
  prev_sample_1 = true; // flag set to true here so for the second sample onwards, line 436 onwards should be executed
  return;
  }
  else {
    new_power_sample = power_sample; 
  }
  slice_area = ((prev_power_sample + new_power_sample)/2)*1; // the height, that is, interval between samples is 1 s. area of trapezium.
  recovered_energy += slice_area; // aggregate area of the trapeziums
  prev_power_sample = new_power_sample;
}
// if power not negative then we stop integrating
else {
  if (recovered_energy <= 0){
    recovered_energy = 0;
  } else {
  power_sample = power;
  if (prev_sample_2 == false){
  prev_power_sample = power_sample;
  prev_sample_2 = true;
  return;
  }
  else {
    new_power_sample = power_sample; 
  }
  slice_area = ((prev_power_sample + new_power_sample)/2)*1;
  recovered_energy -= slice_area;
  prev_power_sample = new_power_sample;
  }
}
  recovered_energy_can = recovered_energy;
}

void updateODentries(){
    // iterate through all name-value pairs in the buffer. Find voltage, current, SOC, power in the buffer and assign the values to the variables in the object dictionary of the node.
    // find voltage, current, etc and update the OD variables with the values.
    for (int i = 0; i < myparser.veEnd; ++i) {
       if (strcmp(myparser.veData[i].veName, "V") == 0){
        voltage = atoi(myparser.veData[i].veValue);
       }
       else if (strcmp(myparser.veData[i].veName, "I") == 0){
        current = atoi(myparser.veData[i].veValue);
        current_can = current + 1000000; // offset by a million. If current is - 200,000 mA then it will be sent as 800,000 mA. At receiving node, subtract current value by 1000000.
       }
       else if (strcmp(myparser.veData[i].veName, "SOC") == 0){
        state_of_charge = atoi(myparser.veData[i].veValue);     // needs to be divided by 10 to get it in percentage. 995/10 = 99.5%.
       }
       else if (strcmp(myparser.veData[i].veName, "P") == 0){
        power = atoi(myparser.veData[i].veValue);
        power_can = power + 1000000; // subtract 1000000 from power on receiving end
        findRecoveredEnergy(); 
       }
  }
  myparser.clearData();
}

void loop(){
  //User Code begin loop() ----------------------------------------------------
  nodeOperatingMode = 0x01;
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    handleCAN(nodeID); // heartbeat is handled in handleCAN.
  }
  
  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    handleCAN(nodeID);
    // In pre-op state, check if data is being read from the shunt, parsed, and stored correctly in the buffer 
    ReadVEData(); // this function passes each incoming byte into rxData. rxData stores the name-value pairs in the buffer (array of structs - 1 struct is 1 name-value pair).
    EverySecond(); // Debug: print the data in the buffer every second.
    if (myparser.isDataAvailable()) {
      updateODentries();  
      last_data_received_time = millis();
    } else {
    currentMillis = millis();
    if (currentMillis - last_data_received_time >= shunt_data_interval) { // if more than 1.5s pass and still no data in buffer then raise error because a new block should be received every sec.
      sendEMCY(1,nodeID, 0x00000301); 
      Serial.println("No Data in the buffer!");
      last_data_received_time = currentMillis;
      }
    }
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    handleCAN(nodeID);
    ReadVEData(); // this function passes each incoming byte into rxData. rxData stores the name-value pairs in the buffer (array of structs - 1 struct is 1 name-value pair).
    EverySecond();
    if (myparser.isDataAvailable()) { // Update OD entries every s. A new block is received every second.
        updateODentries();   
      }
    }
  //User code end loop() --------------------------------------------------------
}






























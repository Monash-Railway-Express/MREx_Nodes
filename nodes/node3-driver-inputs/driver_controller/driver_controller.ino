#include <CAN_MREx.h>
#include <controller.h>

/**
 * @file Controller.ino
 * @brief Driver controller code that controls the inputs recieved from buttons, switches, potentiometers and appropriately sends over CAN
 *
 * @details // TODO
 *
 * @author Chiara Gillam
 * @author Audrey Tasevki
 * @author Kang Yee
 * @author Nicholas Rowe
 * @author Aditya Dinesh Kumar
 *
 * @date Created: 05/08/2025
 *
 * @version 1.2.1
 *
 * @organisation MREX
 *
 * @see //TO DO 
*/

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
uint8_t nodeID = 3;


// --- OD definitions ---

// OD 0x3012:00 – Regen Brake values (pots) (0–1023). <RW>. Mapped to <TPDO1>.
uint16_t od_regen_brake = 0;

// OD 0x606A:00 - Desired speed of motors (pots) (0-1023). <RW>. Mapped to <TPDO1>. 
uint16_t od_motor_command = 0;

// OD 0x6060:00 - Selecting locomotive travel direction mode (switch) (1-back, 2-neutral, 3-forward). <RW>. 
uint8_t od_direction_mode = 0;

// OD 0x6061:00 - Traction condition selector (switch) (int values 1 to 5, 5 being the slipperist). <RW>. 
uint8_t od_condition_mode = 0;

// OD 0x6062:00 - Challenge operation type (switch) (1-throttle control, 2-speed control, 3-autostop, 4-regen, 5-traction). <RW>. 
uint8_t od_challenge_mode = 0;

// OD 0x6065:00 - Sounding horn (button) (1-play sound, 0-default). <RW>. 
uint8_t od_horn_toggle = 0;

//Free 
uint8_t od_button_2 = 0;

// OD 0x3012:01 - Electromagnetic Brake Toggle (switch) (1-on, 0-off). <RW>. 
uint8_t od_service_brake = 0; //parking (1)=on and (0)=off

//Free
uint8_t od_switch_2 = 0;  

// ---------- ADC / filtering settings ----------
const int ADC_RES_BITS = 10;         // 0..1023
const int ADC_SAMPLES  = 20;         // more averaging for 100k sources
const int POT_DEADBAND = 10;         // print only if changed enough


// ---------- Expected raw ADC levels ----------
// 3-position ladder, 4 equal resistors:
// taps are roughly 1/4, 2/4, 3/4 of Vref
const int THREE_LEVELS[3] = {256, 512, 768};

// 5-position ladder, 6 equal resistors:
// taps are roughly 1/6, 2/6, 3/6, 4/6, 5/6 of Vref
const int FIVE_LEVELS[5] = {171, 341, 512, 682, 853};

//Timing for a non blocking function occuring every two seconds
unsigned long previousMillis = 0;
const long interval = 100; // 100 milliseconds


///////////////////SET UP/////////////////////////

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  analogReadResolution(ADC_RES_BITS);

  //Inputs from componments  
  pinMode(BRAKE_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);

  pinMode(DIRECTION_MODE_PIN, INPUT);
  pinMode(OP_MODE_PIN, INPUT);
  pinMode(CHALLENGE_MODE_PIN, INPUT);
  pinMode(CONDITION_MODE_PIN, INPUT);

  //For Sampling
  analogSetPinAttenuation(BRAKE_PIN, ADC_11db);
  analogSetPinAttenuation(THROTTLE_PIN, ADC_11db);
  analogSetPinAttenuation(DIRECTION_MODE_PIN, ADC_11db);
  analogSetPinAttenuation(OP_MODE_PIN, ADC_11db);
  analogSetPinAttenuation(CHALLENGE_MODE_PIN, ADC_11db);

  
  // Initialize CANMREX protocol
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
  registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);
  registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode), &od_direction_mode);
  registerODEntry(0x6061, 0x00, 2, sizeof(od_condition_mode), &od_condition_mode);
  registerODEntry(0x6062, 0x00, 2, sizeof(od_challenge_mode), &od_challenge_mode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
    {0x60FF, 0x00, 16},
    {0x3012, 0x00, 16},
    {0x6062, 0x00, 8}
  };
  mapTPDO(0, tpdoEntries, 2);
  // --- Register RPDOs ---
  // User code Setup end ---------------------------------------------------------
}

////////////////MAIN LOOP//////////////////

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //TO DO: Add emcy vaildation!!
    UpdateOpMode();

    switch (nodeOperatingMode) {
      case MODE_STOPPED: StoppedMode(); break;

      case MODE_PREOP: PreOpMode(); break;

      case MODE_OPERATIONAL: OperationalMode(); break;

      default: StoppedMode(); break; // fail-safe
    }
    // if (nodeOperatingMode == 0x80) { 
    //   HandleDirection();
    // }

    // // --- Operational state ---
    // if (nodeOperatingMode == 0x01) { 

    //   //HandleHorn(); 
    //   HandleInputs();
    //   //Print_Status();
    //   HandleParking();
    // }
  }
}

/////////////MAIN LOOP END//////////////////////

/**
* @brief Function that is called when the system is in stopped mode, Prints Stopped Mode to console
*/
void StoppedMode(){
  Serial.println("Stopped Mode");
}
/**
* @brief Pre Operational Function, calls HandleDirection and HandleChallenge
*/
void PreOpMode(){
  //Serial.println("Pre-Op Mode");
  HandleDirection();
  HandleChallenge();
  //HandleParking();
  //HandleHorn();
}

/**
*@brief OperationMode function - Calls HandleChallenge and HandleInputs
*/
void OperationalMode(){
  //Serial.println("Op Mode");
  HandleChallenge();
  //HandleParking();
  //HandleHorn();
  HandleInputs();
}


/**
 * @brief function that compares current to previous operational modes - if different will send to all nodes  
 */
void UpdateOpMode(){

  // TO DO: Audrey Update 3 pos and 5 pos logic
  int newOpModeRaw = readStable3Pos(OP_MODE_PIN);
  
  //Converting states 1-3 to enum OperatingMode
  uint8_t enumOpMode;

  switch (newOpModeRaw) {
    case 1: enumOpMode = MODE_STOPPED; break;
    case 2: enumOpMode = MODE_PREOP; break;
    case 3: enumOpMode = MODE_OPERATIONAL; break;
    default: return;
  }

  // Checking current mode is different to new
  if(nodeOperatingMode != enumOpMode){
    // Send command to all nodes
    nodeOperatingMode = enumOpMode;  
    Serial.println(nodeOperatingMode);
    // Update local state
    //SendAllNMT(enumOpMode);
  }
}


/**
 * @brief function that is called to send NMT to all nodes
 *
 * @param operatingMode  Current operating mode (will be 0x01, 0x02 or 0x80)
 *
 */
void SendAllNMT(uint8_t operatingMode) {
  //ID's have been changed to variable names to aid readability
  sendNMT(operatingMode, MOTOR_ID); // motor previously 0x01
  sendNMT(operatingMode, BRAKES_ID); // brakes previously 0x02
  sendNMT(operatingMode, LIGHTS_ID); // lights previously 0x04
  sendNMT(operatingMode, AUDIO_ID); // audio sys previously 0x05
  sendNMT(operatingMode, LCD_ID); // LCD screen previously 0x09
}

/**
 * @brief function where all inputs are read and written to the OD variables
 */

void HandleInputs() {
  // ===== Potentiometer Inputs =====
  int od_regen_brake = 1023 - readADC_HighZ(BRAKE_PIN);
  int od_motor_command = readADC_HighZ(THROTTLE_PIN);

  Serial.print("Brake: ");
  Serial.print(od_regen_brake);
  Serial.print("   ||   Throttle: ");
  Serial.println(od_motor_command);

}


/**
 * @brief function that does edge detection on horn button and calls SDO write to horn node
 */
void HandleHorn() {
  Serial.print("Horn Handle: ");
  int newHornToggle = digitalRead(BUTTON_1_PIN);
  Serial.println(newHornToggle);
  if (od_horn_toggle != newHornToggle) {
    od_horn_toggle = newHornToggle;
    uint8_t invertedBtn1 = (uint8_t)!od_horn_toggle;
    //executeSDOWrite(nodeID, AUDIO_ID, 0x6065, 0x00, sizeof(od_horn_toggle), &invertedBtn1);
  }
}

/**
 * @brief Reads the switch which controls the parking break, if the new digital read does not equal the old the parking is sent to the brakes and motors)
 */
void HandleParking() {
  Serial.print("Parking Handle: ");
  int newServiceBrake = digitalRead(SWITCH_1_PIN);
  Serial.println(newServiceBrake);
  if (od_service_brake != newServiceBrake) {
    
    //1 is brake on - 0 is off
    od_service_brake = newServiceBrake;
    //executeSDOWrite(nodeID, BRAKES_ID, 0x3012, 0x01, sizeof(od_service_brake), &od_service_brake);
    //executeSDOWrite(nodeID, MOTOR_ID, 0x3012, 0x01, sizeof(od_service_brake), &od_service_brake);
    Serial.print("Sending Parking");
    Serial.println(od_service_brake);
  }
}

/**
*@brief HandleDirection function reads the desired direction from the 3 positions switch and sends relevent direction to motor and lights
*/
void HandleDirection() {
  Serial.print("Direction Handle: ");
  int newDirectionMode = readStable3Pos(DIRECTION_MODE_PIN);
  Serial.println(newDirectionMode);
  if (od_direction_mode != newDirectionMode && newDirectionMode > 0) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    od_direction_mode = newDirectionMode;
    executeSDOWrite(nodeID, MOTOR_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    executeSDOWrite(nodeID, LIGHTS_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);

    Serial.print("Sending direction");
    Serial.println(od_direction_mode);
  }
}

/**
*@brief Function reads challange 5 position switch, checks if the challenge has changed, and if so writes new challenge to motors. 
*/
void HandleChallenge() {
  Serial.print("Challenge Handle");
  int newChallengeMode = readStable5Pos(CHALLENGE_MODE_PIN);
  Serial.println(newChallengeMode);
  if (od_challenge_mode != newChallengeMode) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    od_challenge_mode = newChallengeMode;
    executeSDOWrite(nodeID, MOTOR_ID, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
    Serial.print("Sending Challenge");
    Serial.println(od_challenge_mode);
  }
}
/**
*@brief Function reads the condition pin, if new state is different to previous state the SDO for the condition is sent to the motor. 
*/
void HandleCondition(){
  Serial.print("Condition Handle");
  int newConditionMode = readStable5Pos(CONDITION_MODE_PIN);
  Serial.println(newConditionMode);
  //Only sends the new condition object as an SDO if there has been a change in the condition. 
  if(od_condition_mode != newConditionMode){
    od_condition_mode = newConditionMode;
    executeSDOWrite(nodeID,MOTOR_ID,0x6061,0x00,sizeof(od_condition_mode),&od_condition_mode);
    Serial.print("Sending Condition");
    Serial.println("od_condition_mode");
    
  }
}


/**
*@brief Helper: Better ADC read for 100l sources
*
*@param pin The pin to read analog value from
*@param samples the number of samples to be taken
*
*@return returns the aveage of the samples as an integer. 
*/
int readADC_HighZ(int pin, int samples) {
  // Let ADC mux settle on this pin
  analogRead(pin);
  delayMicroseconds(500);

  // Throw away a few reads
  analogRead(pin);
  delayMicroseconds(300);
  analogRead(pin);
  delayMicroseconds(300);

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(250);
  }

  return (int)(sum / samples);
}

/**
*@brief Recieves raw analog value and finds the nearest setting (1-3)
*
*@param raw The raw ADC read
*
*@return returns the closest index (1-3)
*/
int decodeNearest3(int raw) {
  int bestIndex = 0;
  int bestErr = abs(raw - THREE_LEVELS[0]);

  for (int i = 1; i < 3; i++) {
    int err = abs(raw - THREE_LEVELS[i]);
    if (err < bestErr) {
      bestErr = err;
      bestIndex = i;
    }
  }

  return bestIndex + 1;  // states 1..3
}

/**
*@brief Recives raw analaog read value and outputs the nearest position, returning 1-5
*
*@param raw Raw value from analog read to be converted to position 1-5 
*
*@return Returns the nearest position 1-5. 
*/
int decodeNearest5(int raw) {
  int bestIndex = 0;
  int bestErr = abs(raw - FIVE_LEVELS[0]);
  for (int i = 1; i < 5; i++) {
    int err = abs(raw - FIVE_LEVELS[i]);
    if (err < bestErr) {
      bestErr = err;
      bestIndex = i;
    }
  }

  return bestIndex + 1;  // states 1..5
}


/**
*@brief Stable selector read for 3 position rotary switch
*
*@param pin this is the pin on the ESP32 to be read from 
*
*@return returns the apropriate position, if three reads do not align, -1 is returned. 
*/
int readStable3Pos(int pin) {
  int r1 = readADC_HighZ(pin);
  int r2 = readADC_HighZ(pin);
  int r3 = readADC_HighZ(pin);

  int s1 = decodeNearest3(r1);
  int s2 = decodeNearest3(r2);
  int s3 = decodeNearest3(r3);

  if (s1 == s2 && s2 == s3) return s1;
  return -1;
}

/**
*@brief Function reads the position of the 5 position switch 
*
*@param pin pin to be read from on the ESP32
*
*@return returns the position 1-5 only if three values line up, if not -1 is returned. 
*/
int readStable5Pos(int pin) {
  int r1 = readADC_HighZ(pin);
  int r2 = readADC_HighZ(pin);
  int r3 = readADC_HighZ(pin);

  int s1 = decodeNearest5(r1);
  int s2 = decodeNearest5(r2);
  int s3 = decodeNearest5(r3);

  if (s1 == s2 && s2 == s3) return s1;
  return -1;
}
#include <CAN_MREx.h>
#include <controller.h>
#include <stdlib.h>
#include "../../../shared/DualSerial/DualSerial.cpp"

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
 * @version 1.3.0
 *
 * @organisation MREX
 *
 * @see //TO DO 
*/

/**
TODO: Change sampling method for switches to use a rolling buffer rather than delays
*/

// User code begin: ------------------------------------------------------

// --- OD definitions ---

// OD 0x3012:00 – Regen Brake values (pots) (0–1023). <RW>. Mapped to <TPDO1>.
uint16_t od_regen_brake = 0;

// OD 0x606A:00 - Desired speed of motors (pots) (0-1023). <RW>. Mapped to <TPDO1>. 
uint16_t od_motor_command = 0;

// OD 0x6060:00 - Selecting locomotive travel direction mode (switch) (1-back, 2-neutral, 3-forward). <RW>. 
uint8_t od_direction_mode = 3;

// OD 0x6061:00 - Traction condition selector (switch) (int values 1 to 5, 5 being the slipperist). <RW>. 
uint8_t od_condition_mode = 0;

// OD 0x6062:00 - Challenge operation type (switch) (1-throttle control, 2-speed control, 3-autostop, 4-regen, 5-traction). <RW>. 
uint8_t od_challenge_mode = 0;

// OD 0x6065:00 - Sounding horn (button) (1-play sound, 0-default). <RW>. 
uint8_t od_horn_toggle = 0;

//Free 
uint8_t od_button_2 = 0;

// OD 0x3012:02 - Electromagnetic Brake Toggle (switch) (1-on, 0-off). <RW>. 
uint8_t od_service_brake_dc = 0; //parking (1)=on and (0)=off

//Free
uint8_t od_switch_2 = 0;  



//Timing for a non blocking function occuring every two seconds
unsigned long previousMillis = 0;
const long interval = 100; // 100 milliseconds

ADCBuffer throttleBuf = {0}; 
ADCBuffer brakeBuf = {0}; 
ADCBuffer dirBuf = {0}; 
ADCBuffer challengeBuf = {0}; 
ADCBuffer conditionBuf = {0};
ADCBuffer opModeBuf = {0};


///////////////////SET UP/////////////////////////

void setup() {
  DualSerial.begin(115200);
  delay(1000);
  DualSerial.println("DualSerial Coms started at 115200 baud");
  analogReadResolution(ADC_RES_BITS);

  //Inputs from componments  
  pinMode(BRAKE_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  
  pinMode(HORN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(SERVICE_BRAKE_PIN, INPUT_PULLUP);
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

  InitBuffer(&throttleBuf, THROTTLE_PIN);
  InitBuffer(&brakeBuf, BRAKE_PIN);
  InitBuffer(&dirBuf, DIRECTION_MODE_PIN);
  InitBuffer(&challengeBuf, CHALLENGE_MODE_PIN);
  InitBuffer(&conditionBuf, CONDITION_MODE_PIN);
  InitBuffer(&opModeBuf, OP_MODE_PIN);
  
  // Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

  xTaskCreatePinnedToCore(
    CAN_Task,
    "CAN Task",
    4096,
    &NODE_ID,   // <--- passed into pvParameters
    3,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(  
    InputTask,  
    "Input Task",  
    4096,  
    NULL, 
    2,          // priority (lower than CAN if needed)  
    NULL,  
    1           // <-- Core 1 
  );

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x606A, 0x00, 2, sizeof(od_motor_command), &od_motor_command);
  registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);
  registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode), &od_direction_mode);
  registerODEntry(0x6061, 0x00, 2, sizeof(od_condition_mode), &od_condition_mode);
  registerODEntry(0x6062, 0x00, 2, sizeof(od_challenge_mode), &od_challenge_mode);
  registerODEntry(0x6065, 0x00, 2, sizeof(od_horn_toggle), &od_horn_toggle);
  registerODEntry(0x3012, 0x02, 2, sizeof(od_service_brake_dc), &od_service_brake_dc);


  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + NODE_ID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
    {0x3012, 0x00, 16},    // regen brake
    {0x606A, 0x00, 16},   // motor command
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

    /**
    TODO: Add emcy validation
    can't change op mode during major emergency
    */

    UpdateOpMode();

    switch (nodeOperatingMode) {
      case MODE_STOPPED: StoppedMode(); break;

      case MODE_PREOP: PreOpMode(); break;

      case MODE_OPERATIONAL: OperationalMode(); break;

      default: StoppedMode(); break; // fail-safe
    }

    DualSerial.println(" ");
  }
}

/////////////MAIN LOOP END//////////////////////

/**
* @brief Function that is called when the system is in stopped mode, Prints Stopped Mode to console
*/
void StoppedMode(){
  DualSerial.println("Stopped Mode");
  /**
  TODO: Check speed/throttle, regen brakes and service brake status and send minor emergencies accordingly.
  */
}

/**
* @brief Pre Operational Function, calls HandleDirection and HandleChallenge
*/
void PreOpMode(){
  DualSerial.print("Pre-Op Mode");
  HandleDirection();
  HandleChallenge();
  HandleParking();
  HandleHorn();
}

/**
*@brief OperationMode function - Calls HandleChallenge and HandleInputs
*/
void OperationalMode(){
  DualSerial.print("Op Mode");
  HandleChallenge();
  HandleParking();
  HandleHorn();
  HandleInputs();
}


/**
 * @brief function that compares current to previous operational modes - if different will send to all nodes  
 */
void UpdateOpMode(){

  int newOpModeRaw = ReadStable3PosBuffered(&opModeBuf);
  
  //Converting states 1-3 to enum OperatingMode

  uint8_t enumOpMode = opModes[newOpModeRaw];


  // Checking current mode is different to new
  if(nodeOperatingMode != enumOpMode){
    // Send command to all nodes
    nodeOperatingMode = enumOpMode;  
    DualSerial.print(nodeOperatingMode);
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
  sendNMT(operatingMode, AUTOSTOP_ID);
  sendNMT(operatingMode, BATTERY_ID);     
  sendNMT(operatingMode, LCD_ID); // LCD screen previously 0x09
}




/**
 * @brief function where all inputs are read and written to the OD variables
 */
void HandleInputs() {
  // ===== Potentiometer Inputs =====
  uint16_t motorCommand = 1023 - GetAverage(&throttleBuf);  
  od_regen_brake = 1023 - GetAverage(&brakeBuf);

  if (od_service_brake_dc) { // 1, not braking
    od_motor_command = motorCommand;
  } else { // 0, braking
    od_motor_command = 0;
  }

  DualSerial.print("   ||   Brake: ");
  DualSerial.print(od_regen_brake);
  DualSerial.print("   ||   Throttle: ");
  DualSerial.print(od_motor_command);

}





/**
 * @brief function that does edge detection on horn button and calls SDO write to horn node
 */
void HandleHorn() {
  DualSerial.print("   ||   Horn Handle: ");
  int newHornToggle = !(digitalRead(HORN_PIN));
  DualSerial.print(newHornToggle);
  if (od_horn_toggle != newHornToggle) {
    od_horn_toggle = newHornToggle;
    executeSDOWrite(NODE_ID, AUDIO_ID, 0x6065, 0x00, sizeof(od_horn_toggle), &od_horn_toggle);
  }
}






/**
 * @brief Reads the switch which controls the parking break, if the new digital read does not equal the old the parking is sent to the brakes and motors)
 */
void HandleParking() {
  DualSerial.print("   ||   Parking Handle: ");
  int newServiceBrake = digitalRead(SERVICE_BRAKE_PIN);
  DualSerial.print(newServiceBrake);
  if (od_service_brake_dc != newServiceBrake) {
    
    //1 is not braking - 0 is braking
    od_service_brake_dc = newServiceBrake;
    executeSDOWrite(NODE_ID, BRAKES_ID, 0x3012, 0x02, sizeof(od_service_brake_dc), &od_service_brake_dc);
    DualSerial.print("Sending Parking");
    DualSerial.println(od_service_brake_dc);

  }
}





/**
*@brief HandleDirection function reads the desired direction from the 3 positions switch and sends relevent direction to motor and lights
*/
void HandleDirection() {
  DualSerial.print("   ||   Direction Handle: ");
  int newDirectionMode = ReadStable3PosBuffered(&dirBuf);
  DualSerial.print(newDirectionMode);
  if ((od_direction_mode != newDirectionMode) && (newDirectionMode > 0)) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    od_direction_mode = newDirectionMode;
    executeSDOWrite(NODE_ID, MOTOR_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    executeSDOWrite(NODE_ID, LIGHTS_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);

    DualSerial.print("Sending direction");
    DualSerial.println(od_direction_mode);
  }
}




/**
*@brief Function reads challange 5 position switch, checks if the challenge has changed, and if so writes new challenge to motors. 
*/
void HandleChallenge() {
  DualSerial.print("   ||   Challenge Handle: ");
  int newChallengeMode = ReadStable5PosBuffered(&challengeBuf);
  DualSerial.print(newChallengeMode);
  if ((od_challenge_mode != newChallengeMode) && (newChallengeMode > 0)) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    od_challenge_mode = newChallengeMode;
    executeSDOWrite(NODE_ID, MOTOR_ID, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
    executeSDOWrite(NODE_ID, AUTOSTOP_ID, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
    DualSerial.print("Sending Challenge");
    DualSerial.println(od_challenge_mode);
  }
}





/**
*@brief Function reads the condition pin, if new state is different to previous state the SDO for the condition is sent to the motor. 
*/
void HandleCondition(){
  DualSerial.print("   ||   Condition Handle");
  int newConditionMode = ReadStable5PosBuffered(&conditionBuf);
  DualSerial.print(newConditionMode);
  //Only sends the new condition object as an SDO if there has been a change in the condition. 
  if(od_condition_mode != newConditionMode){
    od_condition_mode = newConditionMode;
    executeSDOWrite(NODE_ID,MOTOR_ID,0x6061,0x00,sizeof(od_condition_mode),&od_condition_mode);
    DualSerial.print("Sending Condition");
    DualSerial.println(od_condition_mode);
    
  }
}

/**
 * @brief Updates the circular ADC sample buffer with a new reading from the specified pin
 *
 * @param buf Pointer to the ADCBuffer to update
 * @param pin The analog pin to read from
 */
void UpdateADCBuffer(ADCBuffer* buf, int pin) {  
  buf->samples[buf->index] = analogRead(pin);  
  buf->index = (buf->index + 1) % BUF_SIZE; 
}

/**
 * @brief Computes the average of all samples currently stored in the ADC buffer
 *
 * @param buf Pointer to the ADCBuffer to average
 *
 * @return Integer average of all samples in the buffer
 */
int GetAverage(ADCBuffer* buf) { 
  int sum = 0;  
  for (int i = 0; i < BUF_SIZE; i++) {    
    sum += buf->samples[i];  
  }  
  return sum / BUF_SIZE; 
}

/**
 * @brief Returns a stable 3-position switch reading by averaging the ADC buffer and decoding to nearest position
 *
 * @param buf Pointer to the ADCBuffer associated with the switch pin
 *
 * @return Decoded switch position (1–3)
 */
int ReadStable3PosBuffered(ADCBuffer* buf) {  
  int avg = GetAverage(buf);  
  return DecodeNearest3(avg); 
}

/**
 * @brief Returns a stable 5-position switch reading by averaging the ADC buffer and decoding to nearest position
 *
 * @param buf Pointer to the ADCBuffer associated with the switch pin
 *
 * @return Decoded switch position (1–5)
 */
int ReadStable5PosBuffered(ADCBuffer* buf) {  
  int avg = GetAverage(buf);  
  return DecodeNearest5(avg); 
}

/**
*@brief Recieves raw analog value and finds the nearest setting (1-3)
*
*@param raw The raw ADC read
*
*@return returns the closest index (1-3)
*/
int DecodeNearest3(int raw) {
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
int DecodeNearest5(int raw) {
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
 * @brief FreeRTOS task that continuously samples all analog input pins into their respective ADC buffers at 200 Hz
 *
 * @param pvParameters Unused task parameter (pass NULL)
 */
void InputTask(void* pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(5); // 200 Hz sampling

  while (true) { 
    UpdateADCBuffer(&throttleBuf, THROTTLE_PIN); 
    UpdateADCBuffer(&brakeBuf, BRAKE_PIN); 
    UpdateADCBuffer(&dirBuf, DIRECTION_MODE_PIN); 
    UpdateADCBuffer(&opModeBuf, OP_MODE_PIN); 
    UpdateADCBuffer(&challengeBuf, CHALLENGE_MODE_PIN); 
    UpdateADCBuffer(&conditionBuf, CONDITION_MODE_PIN); 

  vTaskDelay(delayTicks);
  } 
}

void InitBuffer(ADCBuffer* buf, int pin) {
  for (int i = 0; i < BUF_SIZE; i++) {
    buf->samples[i] = analogRead(pin);
  }
  buf->index = 0;
}



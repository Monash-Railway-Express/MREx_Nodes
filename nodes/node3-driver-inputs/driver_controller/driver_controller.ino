#include <CAN_MREx.h>
#include <controller.h>
#include <stdlib.h>
#include <HardwareSerial.h>  // Nextion UART

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
 * @version 1.2.2
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

// ═══════════════════════════════════════════════════════════════
// NEXTION HMI
// ═══════════════════════════════════════════════════════════════

HardwareSerial nextionSerial(1); // UART1

// Previous value cache
int     prevSpeed       = -1;
int     prevThrottle    = -1;
int     prevBrake       = -1;
int     prevDirection   = -1;
int     prevChallenge   = -1;
int     prevCondition   = -1;
int     prevBrakeStatus = -1;
uint8_t prevOpMode      = 255;

void sendText(String component, String value) {
  nextionSerial.print(component + ".txt=\"" + value + "\"");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendProgressBar(String component, int value) {
  nextionSerial.print(component + ".val=" + String(value));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendColour(String component, String attr, int colour) {
  nextionSerial.print(component + "." + attr + "=" + String(colour));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void refreshComponent(String component) {
  nextionSerial.print("ref " + component);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

#define NEX_GREEN  1339
#define NEX_YELLOW 65504
#define NEX_RED    63488
#define NEX_WHITE  65535
#define NEX_GREY   33808
#define NEX_CYAN   1055
#define NEX_DARK   10

String getDirectionText(int dir) {
  switch (dir) {
    case 1: return "REVERSE";
    case 2: return "NEUTRAL";
    case 3: return "FORWARD";
    default: return "--";
  }
}

int getDirectionColour(int dir) {
  switch (dir) {
    case 1: return NEX_RED;
    case 2: return NEX_YELLOW;
    case 3: return NEX_GREEN;
    default: return NEX_GREY;
  }
}

String getChallengeText(int mode) {
  switch (mode) {
    case 1: return "Throttle Ctrl";
    case 2: return "Speed Ctrl";
    case 3: return "Autostop";
    case 4: return "Regen";
    case 5: return "Traction";
    default: return "--";
  }
}

String getConditionText(int mode) {
  switch (mode) {
    case 1: return "Dry";
    case 2: return "Light Wet";
    case 3: return "Wet";
    case 4: return "Very Wet";
    case 5: return "Extreme";
    default: return "--";
  }
}

String getOpModeText(uint8_t mode) {
  switch (mode) {
    case MODE_STOPPED:     return "Stopped";
    case MODE_PREOP:       return "Pre-Op";
    case MODE_OPERATIONAL: return "Operational";
    default: return "--";
  }
}

int getOpModeColour(uint8_t mode) {
  switch (mode) {
    case MODE_STOPPED:     return NEX_RED;
    case MODE_PREOP:       return NEX_YELLOW;
    case MODE_OPERATIONAL: return NEX_CYAN;
    default: return NEX_GREY;
  }
}

void updateAutostopPill(int challengeMode) {
  if (challengeMode == 3) {
    sendColour("t_autostop", "bco", NEX_GREEN);
    sendColour("t_autostop", "pco", 0);
  } else {
    sendColour("t_autostop", "bco", NEX_DARK);
    sendColour("t_autostop", "pco", NEX_GREY);
  }
  refreshComponent("t_autostop");
}

void updateNextion() {
  // THROTTLE
  int throttlePct = map(od_motor_command, 0, 1023, 0, 100);
  if (throttlePct != prevThrottle) {
    sendText("t_throttle", String(throttlePct) + " %");
    sendProgressBar("j_throttle", throttlePct);
    prevThrottle = throttlePct;
  }

  // BRAKE
  int brakePct = map(od_regen_brake, 0, 1023, 0, 100);
  if (brakePct != prevBrake) {
    sendText("t_brakepct", String(brakePct) + " %");
    sendProgressBar("j_brake", brakePct);
    prevBrake = brakePct;
  }

  // BRAKE STATUS
  if (od_service_brake_dc != prevBrakeStatus) {
    bool applied = (od_service_brake_dc == 0);
    sendText("t_brakestatus", applied ? "Applied" : "Released");
    sendColour("t_brakestatus", "pco", applied ? NEX_RED : NEX_GREEN);
    refreshComponent("t_brakestatus");
    prevBrakeStatus = od_service_brake_dc;
  }

  // DIRECTION MODE
  if (od_direction_mode != prevDirection) {
    sendText("t_direction", getDirectionText(od_direction_mode));
    sendColour("t_direction", "pco", getDirectionColour(od_direction_mode));
    refreshComponent("t_direction");
    prevDirection = od_direction_mode;
  }

  // CHALLENGE MODE
  if (od_challenge_mode != prevChallenge) {
    sendText("t_challenge", getChallengeText(od_challenge_mode));
    updateAutostopPill(od_challenge_mode);
    prevChallenge = od_challenge_mode;
  }

  // CONDITION MODE
  if (od_condition_mode != prevCondition) {
    sendText("t_condition", getConditionText(od_condition_mode));
    prevCondition = od_condition_mode;
  }

  // OPERATION MODE
  if (nodeOperatingMode != prevOpMode) {
    sendText("t_opmode", getOpModeText(nodeOperatingMode));
    sendColour("t_opmode", "pco", getOpModeColour(nodeOperatingMode));
    refreshComponent("t_opmode");
    prevOpMode = nodeOperatingMode;
  }
}

// ═══════════════════════════════════════════════════════════════
// END NEXTION HMI
// ═══════════════════════════════════════════════════════════════


///////////////////SET UP/////////////////////////

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  analogReadResolution(ADC_RES_BITS);

  nextionSerial.begin(115200, SERIAL_8N1, 18, 17); // Nextion UART1

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
    &NODE_ID,
    3,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(  
    InputTask,  
    "Input Task",  
    4096,  
    NULL, 
    2,
    NULL,  
    1
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
  configureTPDO(0, 0x180 + NODE_ID, 255, 100, 100);
  
  PdoMapEntry tpdoEntries[] = {
    {0x3012, 0x00, 16},
    {0x606A, 0x00, 16},
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
      default: StoppedMode(); break;
    }

    updateNextion(); // Update Nextion display

    Serial.println(" ");
  }
}

/////////////MAIN LOOP END//////////////////////

void StoppedMode(){
  Serial.println("Stopped Mode");
}

void PreOpMode(){
  Serial.print("Pre-Op Mode");
  HandleDirection();
  HandleChallenge();
  HandleParking();
  HandleHorn();
}

void OperationalMode(){
  Serial.print("Op Mode");
  HandleChallenge();
  HandleParking();
  HandleHorn();
  HandleInputs();
}

void UpdateOpMode(){
  int newOpModeRaw = ReadStable3PosBuffered(&opModeBuf);
  uint8_t enumOpMode = opModes[newOpModeRaw];
  if(nodeOperatingMode != enumOpMode){
    nodeOperatingMode = enumOpMode;  
    Serial.print(nodeOperatingMode);
    //SendAllNMT(enumOpMode);
  }
}

void SendAllNMT(uint8_t operatingMode) {
  sendNMT(operatingMode, MOTOR_ID);
  sendNMT(operatingMode, BRAKES_ID);
  sendNMT(operatingMode, LIGHTS_ID);
  sendNMT(operatingMode, AUDIO_ID);
  sendNMT(operatingMode, AUTOSTOP_ID);
  sendNMT(operatingMode, BATTERY_ID);     
  sendNMT(operatingMode, LCD_ID);
}

void HandleInputs() {
  uint16_t motorCommand = 1023 - GetAverage(&throttleBuf);  
  od_regen_brake = 1023 - GetAverage(&brakeBuf);
  if (od_service_brake_dc) {
    od_motor_command = motorCommand;
  } else {
    od_motor_command = 0;
  }
  Serial.print("   ||   Brake: ");
  Serial.print(od_regen_brake);
  Serial.print("   ||   Throttle: ");
  Serial.print(od_motor_command);
}

void HandleHorn() {
  Serial.print("   ||   Horn Handle: ");
  int newHornToggle = !(digitalRead(HORN_PIN));
  Serial.print(newHornToggle);
  if (od_horn_toggle != newHornToggle) {
    od_horn_toggle = newHornToggle;
    executeSDOWrite(NODE_ID, AUDIO_ID, 0x6065, 0x00, sizeof(od_horn_toggle), &od_horn_toggle);
  }
}

void HandleParking() {
  Serial.print("   ||   Parking Handle: ");
  int newServiceBrake = digitalRead(SERVICE_BRAKE_PIN);
  Serial.print(newServiceBrake);
  if (od_service_brake_dc != newServiceBrake) {
    od_service_brake_dc = newServiceBrake;
    executeSDOWrite(NODE_ID, BRAKES_ID, 0x3012, 0x02, sizeof(od_service_brake_dc), &od_service_brake_dc);
    Serial.print("Sending Parking");
    Serial.println(od_service_brake_dc);
  }
}

void HandleDirection() {
  Serial.print("   ||   Direction Handle: ");
  int newDirectionMode = ReadStable3PosBuffered(&dirBuf);
  Serial.print(newDirectionMode);
  if ((od_direction_mode != newDirectionMode) && (newDirectionMode > 0)) {
    od_direction_mode = newDirectionMode;
    executeSDOWrite(NODE_ID, MOTOR_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    executeSDOWrite(NODE_ID, LIGHTS_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    Serial.print("Sending direction");
    Serial.println(od_direction_mode);
  }
}

void HandleChallenge() {
  Serial.print("   ||   Challenge Handle: ");
  int newChallengeMode = ReadStable5PosBuffered(&challengeBuf);
  Serial.print(newChallengeMode);
  if ((od_challenge_mode != newChallengeMode) && (newChallengeMode > 0)) {
    od_challenge_mode = newChallengeMode;
    executeSDOWrite(NODE_ID, MOTOR_ID, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
    executeSDOWrite(NODE_ID, AUTOSTOP_ID, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
    Serial.print("Sending Challenge");
    Serial.println(od_challenge_mode);
  }
}

void HandleCondition(){
  Serial.print("   ||   Condition Handle");
  int newConditionMode = ReadStable5PosBuffered(&conditionBuf);
  Serial.print(newConditionMode);
  if(od_condition_mode != newConditionMode){
    od_condition_mode = newConditionMode;
    executeSDOWrite(NODE_ID,MOTOR_ID,0x6061,0x00,sizeof(od_condition_mode),&od_condition_mode);
    Serial.print("Sending Condition");
    Serial.println(od_condition_mode);
  }
}

void UpdateADCBuffer(ADCBuffer* buf, int pin) {  
  buf->samples[buf->index] = analogRead(pin);  
  buf->index = (buf->index + 1) % BUF_SIZE; 
}

int GetAverage(ADCBuffer* buf) { 
  int sum = 0;  
  for (int i = 0; i < BUF_SIZE; i++) {    
    sum += buf->samples[i];  
  }  
  return sum / BUF_SIZE; 
}

int ReadStable3PosBuffered(ADCBuffer* buf) {  
  int avg = GetAverage(buf);  
  return DecodeNearest3(avg); 
}

int ReadStable5PosBuffered(ADCBuffer* buf) {  
  int avg = GetAverage(buf);  
  return DecodeNearest5(avg); 
}

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
  return bestIndex + 1;
}

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
  return bestIndex + 1;
}

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

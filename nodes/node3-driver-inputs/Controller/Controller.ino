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
 * @version 1.0.3
 *
 * @organisation MREX
 *
 * @see //TO DO 
*/

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
uint8_t nodeID = 3;


// --- OD definitions ---
//To Do (Nick): Find apropriate values, direction and PDO mapping for each of the following variables

// OD 0x3012:00 – Regen brake values (pots) (0–1023). <insert R/W/RW>. Mapped to <TPDx>.
uint16_t od_regen_brake = 0;
// OD 0x60FF:00 - Desired speed of motors (pots) (value). <R/W/RW>. Mapped to <TPDx>. 
uint16_t od_desired_speed = 0;
//TO DO (Nick) - Fill in OD Definitions for below and check these are OD's
uint8_t od_button_1 = 0;
uint8_t od_button_2 = 0;
uint8_t od_switch_1 = 0; //parking (1)=on and (0)=off
uint8_t od_switch_2 = 0;  // loc ann
uint8_t od_direction_mode = 0;  
uint8_t od_condition_mode = 0;
uint8_t challenge_mode = 0;
uint8_t operation_mode = 0;
uint8_t od_mc_service_brake_request = 1;

// previous state variables (used for edge detection)
bool b1prev = HIGH; 
bool b2prev = HIGH; 
bool s1prev = HIGH; // parking - initally on (1) 
bool s2prev = HIGH;
int dirprev = 101; 
int opModePrev = 1; 

//Timing for a non blocking function occuring every two seconds
unsigned long previousMillis = 0;
const long interval = 100; // 100 milliseconds


// User code end ---------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  analogReadResolution(10);

  //Inputs from componments  
  pinMode(BRAKE_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);

  pinMode(DIRECTION_MODE_PIN, INPUT);
  pinMode(OP_MODE_PIN, INPUT);
  pinMode(CHALLENGE_MODE_PIN, INPUT);
  pinMode(CONDITION_MODE_PIN, INPUT);
  
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
  registerODEntry(0x60FF, 0x00, 2, sizeof(od_desired_speed), &od_desired_speed);
  registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);
  registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode), &od_direction_mode);
  registerODEntry(0x6061, 0x00, 2, sizeof(od_condition_mode), &od_condition_mode);
  registerODEntry(0x6062, 0x00, 2, sizeof(challenge_mode), &challenge_mode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
    {0x60FF, 0x00, 16},
    {0x3012, 0x00, 16}
  };
  mapTPDO(0, tpdoEntries, 2);
  // --- Register RPDOs ---
  // User code Setup end ---------------------------------------------------------
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int opMode = Map5PosTo3State(analogRead(CONDITION_MODE_PIN));

    // Updating Op mode 
    if((opModePrev != opMode) && (opMode != 101))
    {
      SendToNewOpMode(opMode);
      opModePrev = opMode;
    }

    if (nodeOperatingMode == 0x80) { 
      HandleDirection();
    }

    // --- Operational state ---
    if (nodeOperatingMode == 0x01) { 

      //HandleHorn(); 
      HandleInputs();
      //Print_Status();
      HandleParking();
    }
  }
}

//function definitions
// function to go to stopped / pre-op / operational mode
void SendToNewOpMode(int opMode) {
  //Checks if EMCY 
  //if(!checkMajorEMCY() || !checkMinorEMCY()){

      if (opMode == 1) {
      //previously: SendAllNMT(0x02);
      SendAllNMT(MODE_STOPPED);
      nodeOperatingMode = MODE_STOPPED; //previosuly set to 0x02
      Serial.println("Stopped Mode");
    }
    if (opMode == 3) {
      //previously: SendAllNMT(0x01);
      SendAllNMT(MODE_OPERATIONAL);
      nodeOperatingMode = MODE_OPERATIONAL; //previously set to 0x01
      Serial.println("Normal Mode");
    }
    if (opMode == 2) {
      SendAllNMT(MODE_PREOP); //previously SendAllNMT(0x80)
      nodeOperatingMode = MODE_PREOP; //previously set to 0x80
      Serial.println("Preop Mode");

    }
  //}
  
}

// function that is called to send NMT to all nodes
//ID's have been changed to variable names to aid readability
void SendAllNMT(uint8_t operatingMode) {
  sendNMT(operatingMode, MOTOR_ID); // motor previously 0x01
  sendNMT(operatingMode, BRAKES_ID); // brakes previously 0x02
  sendNMT(operatingMode, LIGHTS_ID); // lights previously 0x04
  sendNMT(operatingMode, AUDIO_ID); // audio sys previously 0x05
  sendNMT(operatingMode, LCD_ID); // LCD screen previously 0x09
}

// function where all inputs are read
void HandleInputs() {
  // ===== Potentiometer Inputs =====
  od_regen_brake   = 1023 - analogRead(BRAKE_PIN);
  od_desired_speed = 1023 - analogRead(SPEED_PIN);
  Serial.print("Brake: ");
  Serial.print(od_regen_brake);
  Serial.print("   ||   Throttle: ");
  Serial.println(od_desired_speed);


  // ===== Button Inputs =====
  od_button_1 = digitalRead(BUTTON_1_PIN);
  od_button_2 = digitalRead(BUTTON_2_PIN);

  // ===== Switch Inputs =====
  od_switch_1 = digitalRead(SWITCH_1_PIN);
  od_switch_2 = digitalRead(SWITCH_2_PIN);

  // Store raw 5-position states too if you want to use them elsewhere later
  
  //od_condition_mode = Check5Switch(analogRead(CONDITION_MODE_PIN));
}

// Used for debugging. Prints all inputs and their values
void PrintStatus() {
  // Check readings of brake and speed
   Serial.print("Speed: ");
   Serial.print(od_desired_speed);
   Serial.print(" | Brake: ");
   Serial.println(od_regen_brake);

  // Check buttons
  // Serial.print(" || Button 1: ");
  // Serial.print(od_button_1);
  // Serial.print(" | Button 2: ");
  // Serial.println(od_button_2);

  // Check switches
  Serial.print(" || Switch 1: ");
  Serial.print(od_switch_1);
  Serial.print(" | Switch 2: ");
  Serial.println(od_switch_2);


  // Serial.print("Op_mode raw: ");
  // Serial.print(analogRead(OP_MODE_PIN));
  // Serial.print(" | Op_mode pos3: ");
  // Serial.print(Check3Switch(analogRead(OP_MODE_PIN)));

  // Serial.print(" || Direction raw: ");
  // Serial.print(analogRead(DIRECTION_MODE_PIN));
  // Serial.print(" | Condition pos3: ");
  // Serial.print(Check3Switch(analogRead(DIRECTION_MODE_PIN)));

  Serial.print("Challenge raw: ");
  Serial.print(analogRead(CHALLENGE_MODE_PIN));
  Serial.print(" | Challenge pos5: ");
  Serial.print(Map5PosTo3State(analogRead(CHALLENGE_MODE_PIN)));

  // Serial.print(" || Condition raw: ");
  // Serial.print(analogRead(CONDITION_MODE_PIN));
  // Serial.print(" | Condition pos5: ");
  // Serial.print(Check5Switch(analogRead(CONDITION_MODE_PIN)));
  
  // Mapping 3 to 5 - Back up test Code
  // Serial.print(" | OpMode mapped: ");
  // Serial.println(Map5PosTo3State(od_condition_modeRaw));
}


// function that does edge detection on horn button and calls SDO write to horn node
// NOTE: Horn is currently assigned to Button 1
void HandleHorn() {
  if (od_button_1 != b1prev) {
    od_direction_mode = Map5PosTo3State(analogRead(CHALLENGE_MODE_PIN));
    b1prev = od_button_1;
    uint8_t invertedBtn1 = (uint8_t)!od_button_1;
    executeSDOWrite(nodeID, 5, 0x6065, 0x00, sizeof(od_button_1), &invertedBtn1);
  }
}

void HandleParking() {
  if (od_switch_1 != s1prev) {
    //1 is brake on - 0 is off
    s1prev = od_switch_1;
    executeSDOWrite(nodeID, 2, 0x3012, 0x01, sizeof(od_switch_1), &od_switch_1);
    Serial.print("Sending Parking");
    Serial.println(od_switch_1);
  }
}

void HandleDirection() {
  od_direction_mode = Map5PosTo3State(analogRead(CHALLENGE_MODE_PIN));
  // Test values 
  // Serial.println(od_direction_mode);
  // Serial.println(dirprev);
  if ((od_direction_mode != dirprev) && od_direction_mode != 101) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    dirprev = od_direction_mode;
    executeSDOWrite(nodeID, 1, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    Serial.print("Sending direction");
    Serial.println(od_direction_mode);
  }
}

// old 3-position switch checker
int Check3Switch(int read) {
  Serial.println(read);
  if (read < 200) {
    // neutral and pre op
    return 1;
  }
  else if (read > 500) {
    // op and forward
    return 3;
  }
  else {
    // stopped and backward 
    return 2;
  }
}


// 5-position switch checker
int Check5Switch(int read) {
  //Serial.println(read);
  if (read >= 0 && read < 50) {
    return 1;
  }
  else if (read > 100 && read < 250) {
    return 2;
  }
  else if (read > 300 && read < 450) {
    return 3;
  }
  else if (read > 500 && read < 650) {
    return 4;
  }
  else if (read > 700 && read < 850) {
    return 5;
  }
  else {
    return 101;
  }
}


// map 5-position switch into old 3-state behaviour
// positions 1,2 -> state 1
// position 3   -> state 2
// positions 4,5 -> state 3
int Map5PosTo3State(int read) {
  int pos = Check5Switch(read);

  if (pos <= 2) {
    return 1;
  }
  else if (pos == 3) {
    return 2;
  }
  else if (pos == 4 || pos == 5) {
    return 3;
  }
  else {
    return 101;
  }
}



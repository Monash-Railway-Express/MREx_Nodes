#include <CAN_MREx.h>

/**
 * Driver Controller Inputs file
 *
 * File:            Controller.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam, Audrey Tasevki, Kang Yee, Nicholas Rowe, Aditya Dinesh Kumar
 * Date Created:    5/08/2025
 * Last Modified:   4/11/2025
 * Version:         1.0.4
 *
 */

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
uint8_t nodeID = 3;

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_41 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_42 // Set GPIO pins for CAN Receive

#define BRAKE_PIN 14
#define SPEED_PIN 19

#define BUTTON_1_PIN 38   //Horn
#define BUTTON_2_PIN 35   
#define SWITCH_1_PIN 36   //Parking Brake 
#define SWITCH_2_PIN 37   //Location Annoucement

#define DIRECTION_MODE_PIN 5
#define CHALLENGE_MODE_PIN 1
#define CONDITION_MODE_PIN 2
#define OP_MODE_PIN 4

#define HORN_COOLDOWN 500 //DFplayer for horn requires at least 350ms between messages

// --- OD definitions ---
uint16_t od_regen_brake = 0;
uint16_t od_motor_command = 0;
uint8_t button1 = 0;
uint8_t button2 = 0;
uint8_t switch1 = 0; //parking (1)=on and (0)=off
uint8_t switch2 = 0;  // loc ann
uint8_t directionMode = 0;  
uint8_t conditionMode = 0;
uint8_t od_challenge_mode = 0;
uint8_t operationMode = 0;
uint8_t mcServiceBrakeRequest = 1;

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

// Function prototypes
void sendToNewOpMode(int opMode);
void sendAllNMT(uint8_t operatingMode);
void HandleInputs();
void HandleHorn();
void print_status();
int check3Switch(int read);
int check5Switch(int read);
int map5PosTo3State(int read);

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
  registerODEntry(0x606A, 0x00, 2, sizeof(od_motor_command), &od_motor_command);
  registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);
  registerODEntry(0x6060, 0x00, 2, sizeof(directionMode), &directionMode);
  registerODEntry(0x6061, 0x00, 2, sizeof(conditionMode), &conditionMode);
  registerODEntry(0x6062, 0x00, 2, sizeof(od_challenge_mode), &od_challenge_mode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
    {0x606A, 0x00, 16},
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

    int opMode = map5PosTo3State(analogRead(CONDITION_MODE_PIN));

    // Updating Op mode 
    if((opModePrev != opMode) && (opMode != 101))
    {
      sendToNewOpMode(opMode);
      opModePrev = opMode;
    }

    if (nodeOperatingMode == 0x80) { 
      //HandleDirection();
      HandleHorn(); 
    }

    // --- Operational state ---
    if (nodeOperatingMode == 0x01) { 

      HandleHorn(); 
      HandleInputs();
      print_status();
      HandleParking();
    }
  }
}


// function to go to stopped / pre-op / operational mode
void sendToNewOpMode(int opMode) {
  //Checks if EMCY 
  //if(!checkMajorEMCY() || !checkMinorEMCY()){

      if (opMode == 1) {
      sendAllNMT(0x02);
      nodeOperatingMode = 0x02;
      Serial.println("Stopped Mode");
    }
    if (opMode == 3) {
      sendAllNMT(0x01);
      nodeOperatingMode = 0x01;
      Serial.println("Normal Mode");
    }
    if (opMode == 2) {
      sendAllNMT(0x80);
      nodeOperatingMode = 0x80;
      Serial.println("Preop Mode");

    }
  //}
  
}

// function that is called to send NMT to all nodes
void sendAllNMT(uint8_t operatingMode) {
  sendNMT(operatingMode, 0x01); // motor
  sendNMT(operatingMode, 0x02); // brakes
  sendNMT(operatingMode, 0x04); // lights
  sendNMT(operatingMode, 0x05); // audio sys
  sendNMT(operatingMode, 0x09); // LCD screen
}

// function where all inputs are read
void HandleInputs() {
  // ===== Potentiometer Inputs =====
  od_regen_brake   = analogRead(BRAKE_PIN);
  od_motor_command = analogRead(SPEED_PIN);
  Serial.print("Brake: ");
  Serial.print(od_regen_brake);
  Serial.print("   ||   Throttle: ");
  Serial.println(od_motor_command);


  // ===== Button Inputs =====
  button1 = digitalRead(BUTTON_1_PIN);
  button2 = digitalRead(BUTTON_2_PIN);

  // ===== Switch Inputs =====
  switch1 = digitalRead(SWITCH_1_PIN);
  switch2 = digitalRead(SWITCH_2_PIN);

  // Store raw 5-position states too if you want to use them elsewhere later
  
  //conditionMode = check5Switch(analogRead(CONDITION_MODE_PIN));
}

// Used for debugging. Prints all inputs and their values
void print_status() {
  // Check readings of brake and speed
   Serial.print("Speed: ");
   Serial.print(od_motor_command);
   Serial.print(" | Brake: ");
   Serial.println(od_regen_brake);

  // Check buttons
  // Serial.print(" || Button 1: ");
  // Serial.print(button1);
  // Serial.print(" | Button 2: ");
  // Serial.println(button2);

  // Check switches
  Serial.print(" || Switch 1: ");
  Serial.print(switch1);
  Serial.print(" | Switch 2: ");
  Serial.println(switch2);


  // Serial.print("Op_mode raw: ");
  // Serial.print(analogRead(OP_MODE_PIN));
  // Serial.print(" | Op_mode pos3: ");
  // Serial.print(check3Switch(analogRead(OP_MODE_PIN)));

  // Serial.print(" || Direction raw: ");
  // Serial.print(analogRead(DIRECTION_MODE_PIN));
  // Serial.print(" | Condition pos3: ");
  // Serial.print(check3Switch(analogRead(DIRECTION_MODE_PIN)));

  Serial.print("Challenge raw: ");
  Serial.print(analogRead(CHALLENGE_MODE_PIN));
  Serial.print(" | Challenge pos5: ");
  Serial.print(check5Switch(analogRead(CHALLENGE_MODE_PIN)));

  // Serial.print(" || Condition raw: ");
  // Serial.print(analogRead(CONDITION_MODE_PIN));
  // Serial.print(" | Condition pos5: ");
  // Serial.print(check5Switch(analogRead(CONDITION_MODE_PIN)));
  
  // Mapping 3 to 5 - Back up test Code
  // Serial.print(" | OpMode mapped: ");
  // Serial.println(map5PosTo3State(conditionModeRaw));
}


// function that does edge detection on horn button and calls SDO write to horn node
// NOTE: Horn is currently assigned to Button 1
void HandleHorn() {
  static long b1Reenable; 
  static long curFrameTime;

  curFrameTime = millis();
  if(curFrameTime >= b1Reenable){
    if (button1 != b1prev) {
    //directionMode = map5PosTo3State(analogRead(CHALLENGE_MODE_PIN));
    b1prev = button1;
    Serial.println("Horn Pressed");
    uint8_t invertedBtn1 = (uint8_t)!button1;
    executeSDOWrite(nodeID, 5, 0x6065, 0x00, sizeof(button1), &invertedBtn1);
    b1Reenable = curFrameTime + HORN_COOLDOWN;
  }
  }
  
}

void HandleParking() {
  if (switch1 != s1prev) {
    //1 is brake on - 0 is off
    s1prev = switch1;
    executeSDOWrite(nodeID, 2, 0x3012, 0x01, sizeof(switch1), &switch1);
    Serial.print("Sending Parking");
    Serial.println(switch1);
  }
}

void HandleDirection() {
  directionModeRaw = check5Switch(analogRead(CHALLENGE_MODE_PIN));
  // Test values 
  // Serial.println(directionMode);
  // Serial.println(dirprev);
  if ((directionModeRaw != dirprev) && directionModeRaw != 101) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    dirprev = directionModeRaw;
    od_challenge_mode = directionModeRaw;
    executeSDOWrite(nodeID, 1, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
    Serial.print("Sending Challenge");
    Serial.println(od_challenge_mode);
  }
}

// old 3-position switch checker
int check3Switch(int read) {
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
int check5Switch(int read) {
  //Serial.println(read);
  if (read >= 0 && read < 70) {
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
int map5PosTo3State(int read) {
  int pos = check5Switch(read);

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
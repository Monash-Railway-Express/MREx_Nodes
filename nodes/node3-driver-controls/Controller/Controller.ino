#include <CAN_MREx.h>

/**
 * CAN MREX main (Template) file 
 *
 * File:            main.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    5/08/2025
 * Last Modified:   1/10/2025
 * Version:         1.10.2
 *
 */

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 3;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_40 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_41 // Set GPIO pins for CAN Receive

#define BRAKE_PIN 1
#define SPEED_PIN 2


#define BUTTON_1_PIN 45   //Horn
#define BUTTON_2_PIN 35
#define SWITCH_1_PIN 36
#define SWITCH_2_PIN 37

#define DIRECTION_MODE_PIN 5
#define CHALLENGE_MODE_PIN 19
#define CONDITION_MODE_PIN 14
#define OP_MODE_PIN 4


// --- OD definitions ---
uint16_t regenBrake=0;
uint16_t desiredSpeed=0;
uint8_t button1=0;
uint8_t button2=0;
uint8_t switch1=0;
uint8_t switch2=0;
uint8_t directionMode=0;
uint8_t conditionMode=0;
uint8_t challengeMode=0;
uint8_t operationMode=0;


//OPTIONAL: timing for a non blocking function occuring every two seconds
unsigned long previousMillis = 0;
const long interval = 100; // 100 milliseconds

//previous state variables (used for edge detection)
bool b1prev = HIGH; 
bool b2prev = HIGH; 
bool s1prev; 
bool s2prev; 

// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  pinMode(BRAKE_PIN,INPUT);
  pinMode(SPEED_PIN,INPUT);
  pinMode(BUTTON_1_PIN,INPUT_PULLUP);
  pinMode(BUTTON_2_PIN,INPUT_PULLUP);
  pinMode(SWITCH_1_PIN,INPUT_PULLUP);
  pinMode(SWITCH_2_PIN,INPUT_PULLUP);
  pinMode(DIRECTION_MODE_PIN,INPUT);
  pinMode(CHALLENGE_MODE_PIN,INPUT);
  pinMode(CONDITION_MODE_PIN,INPUT);
  pinMode(OP_MODE_PIN,INPUT);
  analogReadResolution(10);
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x60FF, 0x00, 2, sizeof(desiredSpeed), &desiredSpeed);
  registerODEntry(0x3012, 0x00, 2, sizeof(regenBrake), &regenBrake);
  registerODEntry(0x6060, 0x00, 2, sizeof(directionMode), &directionMode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
      {0x60FF, 0x00, 16},  // Example: index 0x2000, subindex 1, 16 bits
      {0x3012, 0x00, 16}    // Example: index 0x2001, subindex 0, 8 bits
    };
  mapTPDO(0, tpdoEntries, 2); //TPDO 1, entries, num entries

  // --- Register RPDOs ---
 
  
  // User code Setup end ---------------------------------------------------------
}


void loop() {
  // //User Code begin loop() ----------------------------------------------------
  handleCAN(nodeID);
  //read operation mode state
  int opMode = check3Switch(analogRead(OP_MODE_PIN));
  sendToNewOpMode(opMode);
  
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    Serial.println("Stopped Mode");
    
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    Serial.println("Preop Mode");
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    Serial.println("Normal Mode");

    directionMode = check3Switch(analogRead(DIRECTION_MODE_PIN));
    //Serial.println(directionMode);
    HandleInputs();
    HandleHorn();
    print_status();

  }

  //User code end loop() --------------------------------------------------------
}

//function to go to stopped mode
void sendToNewOpMode(int opMode){
  if(opMode == 1){
      sendAllNMT(0x02);
      nodeOperatingMode = 0x02;
    }
    if(opMode == 3){
      sendAllNMT(0x01);
      nodeOperatingMode = 0x01;
    }
  if(opMode == 2){
      sendAllNMT(0x80);
      nodeOperatingMode = 0x80;
    }
}

//function that is called to send NMT to all nodes.
void sendAllNMT(uint8_t operatingMode){
  //sendNMT(operatingMode, 0x01); //motor
  //sendNMT(operatingMode, 0x04); //lights
  //sendNMT(operatingMode, 0x05); //audio sys
}

//function where all inputs are read
void HandleInputs(){

  //=====Potentiometer Inputs=====
  regenBrake = 0; //analogRead(BRAKE_PIN);
  desiredSpeed = analogRead(SPEED_PIN);

  //=====Button Inputs=====
  button1 = digitalRead(BUTTON_1_PIN);
  button2 = digitalRead(BUTTON_2_PIN);

  //=====Switch Inputs=====
  switch1 = digitalRead(SWITCH_1_PIN);
  switch2 = digitalRead(SWITCH_2_PIN);
}

//function that does edge detection on horn button and calls SDO write to the horn node
//!!!NOTE: Horn is currently assigned to Button 1.!!!
void HandleHorn(){
  if(button1 != b1prev)
  {
    b1prev = button1;
    uint8_t invertedBtn1 = (uint8_t)!button1;
    //Serial.println("Horn Btn Pressed");
    executeSDOWrite(nodeID,5,0x6065,0x00,sizeof(button1),&invertedBtn1);
  }
}

//Used for debugging. Prints all inputs and their values
void print_status(){
  //Check readings of brake and speed
  // Serial.print("Speed: ");
  // Serial.print(desiredSpeed);
  // Serial.print(" | Brake: ");
  // Serial.println(regenBrake);

  //Check buttons
  // Serial.print(" || Button 1: ");
  // Serial.print(button1);
  // Serial.print(" | Button 2: ");
  // Serial.println(button2);

  //Check switches
  // Serial.print(" || Switch 1: ");
  // Serial.print(switch1);
  // Serial.print(" | Switch 2: ");
  // Serial.println(switch2);

  //Check position switches 3-pos
  directionMode = analogRead(DIRECTION_MODE_PIN);
  Serial.print("|| Direction: ");
  Serial.print(directionMode);
  operationMode = analogRead(OP_MODE_PIN);
  Serial.print("| Operation: ");
  Serial.print(operationMode);

  //Check position switches 5-pos
  // conditionMode = analogRead(CONDITION_MODE_PIN);
  // Serial.print("|| Condition: ");
  // Serial.print(conditionMode);
  // challengeMode = analogRead(CHALLENGE_MODE_PIN);
  // Serial.print("| Challenge: ");
  // Serial.println(challengeMode);
  

}

int check3Switch(int read){
  //Serial.println(read);
  if(read<200){
    //neutral and pre op
    return 1;
  }
  else if(read>500){
    //op and forward
    return 3;
  }
  else{
    //stopped and backward 
    return 2;
  }
}
 
 int check5Switch(int read) {
  //Serial.println(read);
  if(read> 10 && read<80){
    return 4;
  }
  else if(read>80 && read<150){
    return 3;
  }
  else if(read>150 && read<200){
    return 2;
  }
  else if(read ==0){
    return 1;
  }
  else {
    return 5;
  }
 }



/**
 * CAN MREX Lighting control file
 *
 * File:            Lights.ino
 * Organisation:    MREX
 * Author:          Aung Hpone Thant, Chiara Gillam, Oscar Boulter
 * Date Created:    5/10/2025
 * Last Modified:   03/02/2025
 * Version:         1.11.0
 *
 *This code is for the lighting node (Node 4 on the loco). 
 *LIGHTS_FWD will be the control for the white lights on the front and red on the back,
 *LIGHTS_REV will be the control for the red lights on the front and white on the back
 *LIGHTS_PREOP control the yellow lights on all faces of the loco.
 */


#include <CAN_MREx.h> // inlcudes all CAN MREX files

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 4;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_4 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_5 // Set GPIO pins for CAN Receive
#define LIGHT_PREOP 17
#define LIGHT_FWD 15
#define LIGHT_REV 16
enum {Off, PreOp, Neutral, Forward, Reverse} driveState = Off;


// --- OD definitions ---
uint32_t dirMode32;
uint8_t dirMode;

uint8_t intTemp; // If we want to log internal tempature and air quality in the future
uint8_t intAir;  

//misc variables
unsigned long nextPollTime; //used for non blocking delay to request the current motor direction from motor controller

// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
 

  // --- Register TPDOs ---
  

  // --- Register RPDOs ---


  // --- Set pin modes ---
  pinMode(LIGHT_PREOP, OUTPUT);
  pinMode(LIGHT_FWD, OUTPUT);
  pinMode(LIGHT_REV, OUTPUT);

  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  unsigned long  currentMillis = millis();
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    handleCAN(nodeID);
    driveState = Off;
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    checkSensors(); // check sensors always
    handleCAN(nodeID);
    driveState = PreOp;
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    checkSensors();
    handleCAN(nodeID);
    //request the state of the motor drive direction every 200ms
    if (currentMillis >= nextPollTime)
    {
      HandleDirStates();
      nextPollTime = currentMillis + 200;
    }

  }
  HandleOpMode();
  //User code end loop() --------------------------------------------------------
}

//handles the direction selector in operational state
void HandleDirStates()
{
  //reads the motor direction from the controller.
  dirMode32 = executeSDORead(nodeID, 3, 0x6060, 0x00); 
  //dirMode = 1;
  //executeSDOWrite(nodeID, 3, 0x6060, 0x00, sizeof(uint8_t), &dirMode);
  dirMode = (uint8_t)dirMode32;

  //switches the drive state based on the motor direction
  //TODO: clarify codes and implement accordingly. Currently using 0 for fwd and 1 for rev
  if(dirMode == 0)
  {
    driveState = Neutral;
  }
  if(dirMode == 1)
  {
    driveState = Forward;
  }
  if(dirMode == 2)
  {
    driveState = Reverse;
  }
}

void HandleOpMode()
{
  switch (driveState)
  {
    case Forward:
      digitalWrite(LIGHT_FWD, HIGH);
      digitalWrite(LIGHT_REV, LOW);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;
    
    case Reverse:
      digitalWrite(LIGHT_FWD, LOW);
      digitalWrite(LIGHT_REV, HIGH);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;

    case PreOp:
      digitalWrite(LIGHT_FWD, LOW);
      digitalWrite(LIGHT_REV, LOW);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;

    case Neutral:
    digitalWrite(LIGHT_FWD, HIGH);
    digitalWrite(LIGHT_REV, HIGH);
    digitalWrite(LIGHT_PREOP, HIGH);
      break;

    case Off:
      digitalWrite(LIGHT_FWD, LOW);
      digitalWrite(LIGHT_REV, LOW);
      digitalWrite(LIGHT_PREOP, LOW);
      break;
  }
}

// Function for Checking the Temperature and Air Quality of the Sensors
// Assume we are using the digital Output of the Smoke Detector
void checkSensors(){

  bool smokeEMCY;
  bool heatEMCY;

  uint16_t temp;

  smokeEMCY = (digitalRead(SMOKE_PIN) == HIGH); 
  
  temp = analogRead(HEAT_PIN);

  /*
  - Turn the thermistor reading into a celsius temperature (will need callibration)
  - Use only integers
  */
  

  if (smokeEMCY){
    Serial.println("Smoke Error!");
    sendEMCY(0, nodeID, 0x00505);
  }

  if (heatEMCY) {
    Serial.println("Temperature Error!");
    sendEMCY(0, nodeID, 0x00506);
  }

}

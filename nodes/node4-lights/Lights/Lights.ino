/**
 * CAN MREX Lighting control file
 *
 * File:            Lights.ino
 * Organisation:    MREX
 * Author:          Aung Hpone Thant, Chiara Gillam, Oscar Boulter
 * Date Created:    5/10/2025
 * Last Modified:   14/05/2026
 * Version:         1.3.0
 *
 *This code is for the lighting node (Node 4 on the loco). 
 *LIGHTS_FWD will be the control for the white lights on the front and red on the back,
 *LIGHTS_REV will be the control for the red lights on the front and white on the back
 *LIGHTS_PREOP control the yellow lights on all faces of the loco.
 */

uint8_t NODE_ID = 4;  // Change this to set your device's node ID
#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include "../../../shared/DualSerial/DualSerial.cpp"

// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // Set GPIO pins for CAN Receive
#define LIGHT_PREOP 17
#define LIGHT_FWD 15
#define LIGHT_REV 16
#define SMOKE_PIN 4 // arbitrary, change values when required
#define TEMPERATURE_F_PIN 5
#define TEMPERATURE_R_PIN 6
enum {Off, PreOp, Neutral, Forward, Reverse} driveState = Off;


// --- OD definitions ---
uint32_t dirMode32;
uint8_t dirMode;

// If we want to log internal tempature and air quality in the future
uint16_t tempF;
uint16_t tempR; 

//misc variables
unsigned long nextPollTime; //used for non blocking delay to request the current motor direction from motor controller



// User code end ---------------------------------------------------------


void setup() {
  DualSerial.begin(115200);
  delay(1000);
  DualSerial.println("DualSerial Coms started at 115200 baud");
  
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
  enableHeartbeatMonitoring(true);
  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x1004, 0x00, 2, sizeof(tempF), &tempF);
  registerODEntry(0x1004, 0x01, 2, sizeof(tempR), &tempR);
  registerODEntry(0x6060, 0x00, 2, sizeof(dirMode), &dirMode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x184 + NODE_ID, 255, 100, 100);

  PdoMapEntry tpdoEntries[] = {
      {0x1004, 0x00, 16},  // Example: index 0x2000, subindex 1, 16 bits
      {0x1004, 0x01, 16}    // Example: index 0x2001, subindex 0, 8 bits
    };
  mapTPDO(0, tpdoEntries, 2);

  // --- Register RPDOs ---


  // --- Set pin modes ---
  pinMode(LIGHT_PREOP, OUTPUT);
  pinMode(LIGHT_FWD, OUTPUT);
  pinMode(LIGHT_REV, OUTPUT);


  //run lights self test. Flash all lights
  LightsSelfTest();

  //permanently turn yellow lights on
  digitalWrite(LIGHT_PREOP, HIGH);
  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  unsigned long  currentMillis = millis();
  
  //checkSensors(); // always check the sensors
  
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    //handleCAN(NODE_ID);
    driveState = Off;
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    //handleCAN(NODE_ID);
    driveState = PreOp;
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    //handleCAN(NODE_ID);
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
  // //reads the motor direction from the controller.
  // dirMode32 = executeSDORead(NODE_ID, 3, 0x6060, 0x00); 
  // //dirMode = 1;
  // //executeSDOWrite(NODE_ID, 3, 0x6060, 0x00, sizeof(uint8_t), &dirMode);
  // dirMode = (uint8_t)dirMode32;

  //switches the drive state based on the motor direction
  //TODO: clarify codes and implement accordingly. Currently using 3 for fwd and 1 for rev.
  if(dirMode == 2)
  {
    driveState = Neutral;
  }
  if(dirMode == 3)
  {
    driveState = Forward;
  }
  if(dirMode == 1)
  {
    driveState = Reverse;
  }
}

//function that flashes all lights on power on. 
void LightsSelfTest()
{
  digitalWrite(LIGHT_PREOP, LOW);
  digitalWrite(LIGHT_FWD, HIGH);
  digitalWrite(LIGHT_REV, LOW);
  delay(200);
  digitalWrite(LIGHT_PREOP, HIGH);
  digitalWrite(LIGHT_FWD, LOW);
  digitalWrite(LIGHT_REV, LOW);
  delay(200);
  digitalWrite(LIGHT_PREOP, LOW);
  digitalWrite(LIGHT_FWD, LOW);
  digitalWrite(LIGHT_REV, HIGH);
  delay(200);
  digitalWrite(LIGHT_PREOP, HIGH);
  digitalWrite(LIGHT_FWD, HIGH);
  digitalWrite(LIGHT_REV, HIGH);
  delay(500);
  digitalWrite(LIGHT_PREOP, LOW);
  digitalWrite(LIGHT_FWD, LOW);
  digitalWrite(LIGHT_REV, LOW);
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
      //digitalWrite(LIGHT_PREOP, LOW);
      break;
  }
}

// Function for Checking the Temperature and Air Quality of the Sensors
// Assume we are using the digital Output of the Smoke Detector
void checkSensors(){

  bool smokeEMCY;
  bool heatF_EMCY;
  bool heatR_EMCY;

  uint16_t tempF;
  uint16_t tempR;

  smokeEMCY = (digitalRead(SMOKE_PIN) == HIGH); 
  
  tempF = analogRead(TEMPERATURE_F_PIN);
  tempR = analogRead(TEMPERATURE_R_PIN);

  /*
  - Turn the thermistor reading into a celsius temperature (will need callibration)
  - Use only integers
  - Assuming MCP970X Thermistor
    Vout = Tc * Ta + V0c

    V0c: 400mV / 500mV

    Tc: 10.0 / 19.5      Depending on model

    Ta: The Temperature (C)
  */
  
  int Voltage0 = 156; // 500mv Converted to 0-1029 Scale
  int Temperature_Coef = 31; // 10mV Converted to 0-1029 Scale

  tempF = ( tempF - Voltage0 ) / Temperature_Coef;

  tempR = ( tempR - Voltage0 ) / Temperature_Coef;


  //Debugging
  DualSerial.println("Temperature:");
  DualSerial.print("  R: ");
  DualSerial.print(tempR);
  DualSerial.print("  F: ");
  DualSerial.print(tempF);

  heatF_EMCY = tempF > 70;
  heatR_EMCY = tempR > 70;

  if (smokeEMCY){
    DualSerial.println("Smoke Error!");
    sendEMCY(0, NODE_ID, 0x00505);
  }

  if (heatF_EMCY) {
    DualSerial.println("Temperature F Error!");
    sendEMCY(0, NODE_ID, 0x00506);
  }

    if (heatR_EMCY) {
    DualSerial.println("Temperature R Error!");
    sendEMCY(0, NODE_ID, 0x00507);
  }

}

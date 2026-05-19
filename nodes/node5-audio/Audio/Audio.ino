

/**
 * @file Audio.ino
 * @brief CAN MREX Audio Control file 
 *
 * @details 
 * Audio controller for horn and announcements. Uses HardwareSerial to interface
 * between ESP32 and DFPlayerMini via UART. Uses DFRobotDFPlayerMini.h
 * 
 * @author Aung Hpone Thant
 *
 * @date 03/04/2026
 *
 * @organisation MREx
 *
 */

/**
TODO: invert horn signal before use PLEASE!!!!!
*/
 /***************************************************
DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/product-1121.html>
 
 ***************************************************
 DFPlayer
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 ****************************************************/

/*CURRENT AUDIO ORDERING: 
(This is in the order they were copied onto the SD card. 
To reorder them, you have to delete and recopy them in the sequence you want. 
File names mean nothing.)
IMPORTANT: 
USE 16 bit .wav files for horn. Using MP3 adds small silences at start and end of clip, making looping sound choppy.
1.) Startup Feedback Sound (Yarra Trams information chime)
2.) Comeng Horn Middle
3.) Comeng Horn End
*/
#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#include <HardwareSerial.h>
#include <map>



// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
uint8_t nodeID = 5;  // Node 5 - Audio
// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_36 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_35 // Set GPIO pins for CAN Receive

#define STATUS_LED 5
#define PREOP_LED 4

// --- OD definitions ---
uint8_t od_horn = 0;
uint8_t od_loc_counter = 0;

//DFPlayer Setup
#define DFBUSY 13 //used to check if DFPlayer is playing tracks or not. Connect to BUSY pin on DFPlayer. If High, its free.

//sounds definition
#define STARTUP_FB_SOUND 1
#define HORN_LOOP_SOUND 2
#define HORN_END_SOUND 3
//location announcement sound numbers
#define LOC_ANN1_START_SOUND 4
#define LOC_ANN2_AUTOSTOP_SOUND 5
#define LOC_ANN3_COMFORT_SOUND 6
#define LOC_ANN4_COMFORT_END_SOUND 7
#define LOC_ANN5_HAVEN_SOUND 8
#define LOC_ANN6_TCN_SOUND 9
#define LOC_ANN7_TCN_END_SOUND 10
#define LOC_ANN8_END_SOUND 11

//location announcement 
/*
1 - Past station Box (Passing)
// Have 2 traction markers here
2 - Autostop activation (Passing)
3 - Ready for Ride comfort (Stand still)
4 - Completing Ride comfort (Stand Still)
5 - Haven on return (Passing)
6 - Ready for traction (Stand Still)
7 - Completion traction (Passing)
8 - Before Head Shunt (Stand Still)
*/
#define LOC_ANN1_START 1
#define LOC_ANN2_AUTOSTOP 4
#define LOC_ANN3_COMFORT 5
#define LOC_ANN4_COMFORT_END 6
#define LOC_ANN5_HAVEN 7
#define LOC_ANN6_TCN 8
#define LOC_ANN7_TCN_END 9
#define LOC_ANN8_END 10

HardwareSerial player_serial(1); //initialise UART 1 of ESP as the serial for the DFPlayer module
DFRobotDFPlayerMini myDFPlayer;


//user variables
// --- Operating Modes ---
enum OperatingMode : uint8_t {
    MODE_STOPPED        = 0x02,
    MODE_PREOP          = 0x80,
    MODE_OPERATIONAL    = 0x01
};
//function prototypes
void printDetail(uint8_t type, int value);
void StoppedMode();
void PreOpMode();
void OperationalMode();
void HandleHorn();


// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);
  xTaskCreatePinnedToCore(
 
      CAN_Task,
 
      "CAN Task",
 
      6144,
 
      &nodeID,
 
      3,
 
      NULL,
 
      0
  
    );
  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x6065, 0x00, 1, sizeof(od_horn), &od_horn);
  registerODEntry(0x1051, 0x00, 1, sizeof(od_loc_counter), &od_loc_counter);
  // --- Register TPDOs ---
  

  // --- Register RPDOs ---


  // --- DFPlayer Setup ---
  player_serial.begin(9600, SERIAL_8N1, /*rx =*/18, /*tx =*/17);
  Serial.println();
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(player_serial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    sendEMCY(0x01,5,0x00000500); //send minor EMCY message when SD card read failed on startup
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  delay(500);
  Serial.println(F("Audio system online."));
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
  delay(500);
  myDFPlayer.play(STARTUP_FB_SOUND);
  // --- Set pin modes ---
  pinMode(DFBUSY, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PREOP_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  // completing operation mode functionality
    switch (nodeOperatingMode) {
        case MODE_STOPPED:     StoppedMode();     break;
        case MODE_PREOP:       PreOpMode();       break;
        case MODE_OPERATIONAL: OperationalMode(); break;
        default:               StoppedMode();     break;  // fail-safe
    }
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    
    
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 

  }

  //User code end loop() --------------------------------------------------------
}

void StoppedMode(){
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(PREOP_LED, LOW);
}

void PreOpMode(){
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(PREOP_LED, HIGH);
  HandleHorn();
}

void OperationalMode(){
  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(PREOP_LED, LOW);
  HandleHorn();
}
/*
* @brief Handles logic for playing the horn. 
*
* @return Nothing
*/
void HandleHorn() {
  static bool hornStatePrev;
  static uint8_t hornStateMNo = 1; //for horn state machin
  hornStatePrev = (bool)od_horn;
  //Serial.print("Horn state:");
  //Serial.println(hornStatePrev);

  switch (hornStateMNo)
  {
    //State 1: Horn Off
    case 1:
      //1--->2
      if(hornStatePrev)
      {
        //play the looping segment of the horn
        myDFPlayer.loop(HORN_LOOP_SOUND);
        hornStateMNo = 2;
      }
      
      break;

    //State 2: Horn Started
    case 2:

      //2--->1
      if(!hornStatePrev)
      {
        myDFPlayer.play(HORN_END_SOUND);
        hornStateMNo = 1;
      }
      
      break;
  } 

}

/*
* @brief Detects change to autostop counter. Every change, play a sound related to current marker
*
* @return Nothing
*/
void HandleAutoStop(){
  static uint8_t prevCounter = 0; //state variable used for tracking changes to the autostop counter

#define LOC_ANN1_START 1
#define LOC_ANN2_AUTOSTOP 4
#define LOC_ANN3_COMFORT 5
#define LOC_ANN4_COMFORT_END 6
#define LOC_ANN5_HAVEN 7
#define LOC_ANN6_TCN 8
#define LOC_ANN7_TCN_END 9
#define LOC_ANN8_END 10
  //play sound every time location counter updates
  if(prevCounter != od_loc_counter){
    prevCounter = od_loc_counter;
    switch (od_loc_counter) {
        case LOC_ANN1_START:     
          myDFPlayer.play(LOC_ANN1_START_SOUND);     
          break;
        case LOC_ANN2_AUTOSTOP:       
          myDFPlayer.play(LOC_ANN2_AUTOSTOP_SOUND);            
          break;
        case LOC_ANN3_COMFORT: 
          myDFPlayer.play(LOC_ANN3_COMFORT_SOUND);     
          break;
        case LOC_ANN4_COMFORT_END: 
          myDFPlayer.play(LOC_ANN4_COMFORT_END_SOUND);     
          break;
        case LOC_ANN5_HAVEN: 
          myDFPlayer.play(LOC_ANN5_HAVEN_SOUND);     
          break;
        case LOC_ANN6_TCN: 
          myDFPlayer.play(LOC_ANN6_TCN_SOUND);     
          break;
        case LOC_ANN7_TCN_END: 
          myDFPlayer.play(LOC_ANN7_TCN_END_SOUND);     
          break;
        case LOC_ANN8_END: 
          myDFPlayer.play(LOC_ANN8_END_SOUND);     
          break;
        default:                  
          break;  // fail-safe
    }  
  }

}

/*
* @brief Receives status messages from DFPlayer and prints them to console. Add link to CAN error messages further down the line.
*
* @return Nothing
*/
// TODO(AP): Add links to CAN MREx error logging (send minor errors on certain messages)
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}
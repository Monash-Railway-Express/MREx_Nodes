

/**
 * CAN MREX Audio Control file 
 * Audio controller for horn and announcements.
 * File:            main.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    5/08/2025
 * Last Modified:   12/1/2026
 * Version:         1.12.1
 *
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
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

/*CURRENT AUDIO ORDERING: 
(This is in the order they were copied onto the SD card. 
To reorder them, you have to delete and recopy them in the sequence you want. 
File names mean nothing.)
IMPORTANT: 
USE 16 bit .wav files for horn. Using MP3 adds small silences at start and end of clip, making looping sound choppy.
1.) Thomas theme song
2.) EDI Comeng horn
3.) Comeng Horn Start
4.) Comeng Horn Middle
5.) Comeng Horn End
6.) Comeng Horn Long Full
*/

#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"




// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 5;  // Node 5 - Audio

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_33 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_32 // Set GPIO pins for CAN Receive

// --- OD definitions ---
uint8_t horn = 0;

//DFPlayer Setup
#define DFBUSY 13 //used to check if DFPlayer is playing tracks or not. Connect to BUSY pin on DFPlayer. If High, its free.
#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/12, /*tx =*/14);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

//user variables
bool HornState;
bool HornSFXRun;
bool prevHornState;
unsigned long nextInputCheck; //time to check for input 

// User code end ---------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(0x6065, 0x00, 1, sizeof(horn), &horn);

  // --- Register TPDOs ---
  

  // --- Register RPDOs ---

  // --- DFPlayer Setup ---
  #if (defined ESP32)
    FPSerial.begin(9600, SERIAL_8N1, /*rx =*/16, /*tx =*/17);
  #else
    FPSerial.begin(9600);
  #endif
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Wallaby Audio Master"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    sendEMCY(0x01,5,0x00000500); //send minor EMCY message when SD card read failed on startup
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  delay(500);
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30

  // --- Set pin modes ---
  pinMode(DFBUSY, INPUT);

  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    handleCAN(nodeID);
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    handleCAN(nodeID);
    if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
    }
    
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    handleCAN(nodeID);
    HornLogic();
    if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
    }
  }

  //User code end loop() --------------------------------------------------------
}

/*handles logic for playing horn.
It plays the start of a horn sfx, loops the middle part as long as the button is held, then when it is released,
plays the end part. Simply done to attempt to make the horn sound more realistic. 
*/
void HornLogic()
{
  //incredibly cobbled together edge detection. If anyone knows a better way, please feel free.
  HornState = (bool)horn;
  if(HornState && !prevHornState)
  {
    HornSFXRun = HIGH;
    //play the start segment of the horn
    myDFPlayer.play(3);
    prevHornState = HornState;
  }
  else if(!HornState && prevHornState)
  {
    HornSFXRun = LOW;
    myDFPlayer.play(5);
    prevHornState = HornState;
  }

  if(HornSFXRun)
  {
    //if DFBUSY pin is High, the player is idling.
    if(digitalRead(DFBUSY))
    {
      myDFPlayer.play(4);
    } 
  }
}

//error reporting script that came with the DFPlayer example code
//TODO: Connect this to report errors onto the EMCY channel in the CANBUS.
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
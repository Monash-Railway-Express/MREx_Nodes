

/*Test Code for sending an SDO Transmit request over the CAN Bus. Currently in use for testing Horn/Audio system prior
to driver controls node being ready. Mimics Node 3.*/



#include <CAN_MREx.h>
// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
const uint8_t nodeID = 3;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // Set GPIO pins for CAN Receive

// --- OD definitions ---

#define b1 23

bool prevHornBtnState;

uint8_t hornState;

unsigned long b1Reenable;

unsigned long curFrameTime;
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
  pinMode(b1, INPUT_PULLUP);

  // User code Setup end ------------------------------------------------------
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == 0x02){ 
    handleCAN(nodeID);
    sendNMT(0x01, 0x05);
    nodeOperatingMode = 0x01; //go straight to operational for testing reasons.
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == 0x80){ 
    handleCAN(nodeID);
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == 0x01){ 
    handleCAN(nodeID);
    handleButtons();
  }
  //User code end loop() --------------------------------------------------------
}

//temporary function that handles button inputs to switch tracks.
void handleButtons()
{
  curFrameTime = millis();
  if(curFrameTime >= b1Reenable)
  {
    if(prevHornBtnState != !digitalRead(b1))
    {
      Serial.print("Horn State Change: ");
      Serial.println(!digitalRead(b1));
      hornState = (uint8_t)(!digitalRead(b1));
      executeSDOWrite(nodeID, 5,0x6065,0x00,sizeof(hornState), &hornState);
      prevHornBtnState = !digitalRead(b1);
    }
    b1Reenable = curFrameTime + 50;
  }
}

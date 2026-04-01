#include <stdint.h>

#ifndef CONTROLLER

#define CONTROLLER

//Defining Function prototypes
void SendToNewOpMode(int opMode);
void SendAllNMT(uint8_t operatingMode);
void HandleInputs();
void PrintStatus();
void HandleHorn();
void HandleParking();
void HandleDirection();
int Check3Switch(int read);
int Check5Switch(int read);
int Map5PosTo3State(int read);

//defining Pins
#define TX_GPIO_NUM GPIO_NUM_41 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_42 // Set GPIO pins for CAN Receive

#define BRAKE_PIN 14
#define SPEED_PIN 19

#define BUTTON_1_PIN 45   //Horn
#define BUTTON_2_PIN 35   
#define SWITCH_1_PIN 36   //Parking Brake 
#define SWITCH_2_PIN 37   //Location Annoucement

#define DIRECTION_MODE_PIN 5
#define CHALLENGE_MODE_PIN 1
#define CONDITION_MODE_PIN 2
#define OP_MODE_PIN 4

//Defining Node ID's
#define MOTOR_ID 0x01
#define BRAKES_ID 0x02
#define LIGHTS_ID 0x04
#define AUDIO_ID 0x05
#define LCD_ID 0x09


//Defining operating mode enum
enum OperatingMode : uint8_t {
    MODE_STOPPED       = 0x02,
    MODE_PREOP         = 0x80,
    MODE_OPERATIONAL   = 0x01
};



#endif // CONTROLLER
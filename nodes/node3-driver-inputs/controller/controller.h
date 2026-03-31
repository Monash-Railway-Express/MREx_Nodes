#include <stdint.h>

#ifndef CONTROLLER

#define CONTROLLER

//Defining Function prototypes
void sendToNewOpMode(int opMode);
void sendAllNMT(uint8_t operatingMode);
void HandleInputs();
void PrintStatus();
void HandleHorn();
void HandleParking();
void HandleDirection();
int check3Switch(int read);
int check5Switch(int read);
int map5PosTo3State(int read);

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


#endif // CONTROLLER
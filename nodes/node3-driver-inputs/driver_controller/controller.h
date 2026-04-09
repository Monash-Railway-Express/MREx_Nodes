#include <stdint.h>

#ifndef CONTROLLER

#define CONTROLLER

//Defining Function prototypes
void SendAllNMT(uint8_t operatingMode);

void UpdateOpMode();
void StoppedMode();
void PreOpMode();
void OperationalMode();

void HandleInputs();
void HandleHorn();
void HandleParking();
void HandleDirection();
void HandleChallenge();

int readADC_HighZ(int pin, int samples = 20);
int decodeNearest3(int raw);
int decodeNearest5(int raw);
int readStable3Pos(int pin);
int readStable5Pos(int pin);


//defining Pins
#define TX_GPIO_NUM GPIO_NUM_41 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_42 // Set GPIO pins for CAN Receive

#define BRAKE_PIN 1
#define THROTTLE_PIN 2

#define BUTTON_1_PIN 6   //Horn
#define BUTTON_2_PIN 35   
#define SWITCH_1_PIN 15   //Parking Brake 
#define SWITCH_2_PIN 37   //Location Annoucement

#define DIRECTION_MODE_PIN 4
#define CHALLENGE_MODE_PIN 19
#define CONDITION_MODE_PIN 20
#define OP_MODE_PIN 5

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
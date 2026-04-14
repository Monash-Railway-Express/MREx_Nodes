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
void HandleCondition();

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

#define HORN_PIN 7   //Horn
#define BUTTON_2_PIN 35   
#define SERVICE_BRAKE_PIN 6   //Parking Brake 
#define SWITCH_2_PIN 37   //Location Annoucement

#define DIRECTION_MODE_PIN 4
#define CHALLENGE_MODE_PIN 19
#define CONDITION_MODE_PIN 20
#define OP_MODE_PIN 5

uint8_t NODE_ID = 3;

//Defining Node ID's
#define MOTOR_ID 0x01
#define BRAKES_ID 0x02
#define DRIVER_ID 0x03
#define LIGHTS_ID 0x04
#define AUDIO_ID 0x05
#define AUTOSTOP_ID 0x06
#define BATTERY_ID 0x07
#define LOGGER_ID 0x08
#define LCD_ID 0x09

//Defining operating mode enum
enum OperatingMode : uint8_t {
    MODE_STOPPED       = 0x02,
    MODE_PREOP         = 0x80,
    MODE_OPERATIONAL   = 0x01
};

const OperatingMode opModes[3] = {MODE_STOPPED, MODE_STOPPED, MODE_PREOP, MODE_OPERATIONAL};

// ---------- ADC / filtering settings ----------
const int ADC_RES_BITS = 10;         // 0..1023       DO NOT F**KING CHANGE!!!!!!
const int ADC_SAMPLES  = 20;         // more averaging for 10k sources
const int POT_DEADBAND = 10;         // print only if changed enough


// ---------- Expected raw ADC levels ----------
// 3-position ladder, 4 equal resistors:
// taps are roughly 1/4, 2/4, 3/4 of Vref
const int THREE_LEVELS[3] = {256, 512, 768};

// 5-position ladder, 6 equal resistors:
// taps are roughly 1/6, 2/6, 3/6, 4/6, 5/6 of Vref
const int FIVE_LEVELS[5] = {171, 341, 512, 682, 853};


#endif // CONTROLLER
#include <stdint.h>

#ifndef CONTROLLER
#define CONTROLLER

// Defining Function prototypes
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

int DecodeNearest3(int raw);
int DecodeNearest5(int raw);

#define BUF_SIZE 15

typedef struct {
    int samples[BUF_SIZE];
    int index;
} ADCBuffer;

void InputTask(void* pvParameters);
int ReadStable5PosBuffered(ADCBuffer* buf);
int ReadStable3PosBuffered(ADCBuffer* buf);
int GetAverage(ADCBuffer* buf);
void UpdateADCBuffer(ADCBuffer* buf, int pin);
void InitBuffer(ADCBuffer* buf, int pin);

// Defining Pins
#define TX_GPIO_NUM GPIO_NUM_40
#define RX_GPIO_NUM GPIO_NUM_41

#define BRAKE_PIN       GPIO_NUM_1
#define THROTTLE_PIN    GPIO_NUM_2

#define HORN_PIN          GPIO_NUM_15
#define EMCY_CLEAR_PIN    GPIO_NUM_15
#define LOCATION_BUTTON_PIN    GPIO_NUM_15
// #define BUTTON_2_PIN   GPIO_NUM_16  // commented out — GPIO16 used for Nextion RX
#define BUTTON_2_PIN      GPIO_NUM_3   // temp reassignment — button unconnected
#define SERVICE_BRAKE_PIN GPIO_NUM_6
#define SWITCH_2_PIN      37

#define DIRECTION_MODE_PIN  GPIO_NUM_4
#define CHALLENGE_MODE_PIN  GPIO_NUM_19
#define OP_MODE_PIN         GPIO_NUM_5

// Nextion UART pins
#define NEXTION_TX_PIN  GPIO_NUM_7
#define NEXTION_RX_PIN  GPIO_NUM_16

uint8_t NODE_ID = 3;

// Defining Node IDs
#define MOTOR_ID    0x01
#define BRAKES_ID   0x02
#define DRIVER_ID   0x03
#define LIGHTS_ID   0x04
#define AUDIO_ID    0x05
#define AUTOSTOP_ID 0x06
#define BATTERY_ID  0x07
#define LOGGER_ID   0x08
#define LCD_ID      0x09

// Defining operating mode enum
enum OperatingMode : uint8_t {
    MODE_STOPPED       = 0x02,
    MODE_PREOP         = 0x80,
    MODE_OPERATIONAL   = 0x01
};

const OperatingMode opModes[4] = {MODE_STOPPED, MODE_STOPPED, MODE_PREOP, MODE_OPERATIONAL};

// ADC / filtering settings
const int ADC_RES_BITS = 10;
const int ADC_SAMPLES  = 20;
const int POT_DEADBAND = 10;

// Expected raw ADC levels
const int THREE_LEVELS[3] = {256, 512, 768};
const int FIVE_LEVELS[5]  = {171, 341, 512, 682, 853};

#endif // CONTROLLER

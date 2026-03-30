#ifndef MOTOR_VEROBOARD_H
#define MOTOR_VEROBOARD_H


// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_4 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_5 // Set GPIO pins for CAN Receive
#define REGEN_BRAKE_PIN GPIO_NUM_13
#define MOTOR_PIN GPIO_NUM_14
#define REVERSING_PIN GPIO_NUM_10

// Enum for operating modes
enum OperatingMode : uint8_t {
    MODE_STOPPED       = 0x02,
    MODE_PREOP         = 0x80,
    MODE_OPERATIONAL   = 0x01
};

// --- Function Prototypes
void StoppedMode();
void PreOpMode();
void OperationalMode();
void DebugOperationalOutput(uint8_t motorpwmValue, uint8_t brakepwmValue, uint8_t service_brake);
void DebugPreOpOutput();


#endif // MOTOR_VEROBOARD_H
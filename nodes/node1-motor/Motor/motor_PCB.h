/**
 * @file motor_PCB.h
 * @brief Header file for PCB motor node.
 *
 * @details
 * Pin definitions, constants, enums, and function prototypes
 * for the PCB motor controller node.
 *
 * @author Chiara Gillam
 * @author Sean Larkin
 *
 * @date 13/03/2026
 *
 * @version 2.0.0
 *
 * @organisation MREX
 *
 * @see PCB_motor_node.ino
 */

#ifndef MOTOR_PCB_H
#define MOTOR_PCB_H

#include <stdint.h>

// --- Node ID ---
const uint8_t NODE_ID = 1;

// --- CAN Pins ---
#define TX_GPIO_NUM GPIO_NUM_48
#define RX_GPIO_NUM GPIO_NUM_47

// --- DAC I2C ---
#define DAC_ADDR  0x48
#define SDA_PIN   GPIO_NUM_2
#define SCL_PIN   GPIO_NUM_1

// --- GPIO Pins ---
#define THROTTLE_SWITCH      GPIO_NUM_4
#define REVERSING_SWITCH_MC  GPIO_NUM_5
#define BRAKE_SWITCH         GPIO_NUM_6
#define MOTOR1_GREEN_LED     GPIO_NUM_7
#define MOTOR2_GREEN_LED     GPIO_NUM_15
#define MOTOR1_RED_LED       GPIO_NUM_16
#define MOTOR2_RED_LED       GPIO_NUM_17
#define ENCODER_A            GPIO_NUM_18
#define ENCODER_B            GPIO_NUM_8
#define ENCODER_Z            GPIO_NUM_9
#define REVERSING_CONTACTOR  GPIO_NUM_10

// --- Pulse Counter ---
#define PCNT_HIGH_LIMIT  32767
#define PCNT_LOW_LIMIT   0

// --- Physical Constants ---
const float PULSES_PER_REV       = 200.0f;
const float WHEEL_CIRCUMFERENCE_M = 0.3f;

// --- Control Limits ---
#define DAC_MAX              1023
#define REGEN_BRAKE_THRESHOLD  10   // Minimum regen brake value to activate
#define LOOP_INTERVAL_MS      10   // Control loop period in ms
#define MAX_SPEED_KMH  15.0f   // Hard speed cap — throttle cut above this speed (km/h)


// --- Speed Thresholds ---
#define SERVICE_BRAKE_SPEED_KMH  2.0f   // Apply service brake below this speed (km/h)
#define AUTO_STOP_SPEED_KMH      5.0f   // Target stop speed for AutoStop challenge (km/h)

// --- Challenge Mode Values ---
#define CHALLENGE_NORMAL          0
#define CHALLENGE_SPEED_CONTROL   1
#define CHALLENGE_AUTO_STOP       2
#define CHALLENGE_TRACTION        3
#define CHALLENGE_ENERGY_RECOVERY 4

// --- Operating Modes ---
enum OperatingMode : uint8_t {
    MODE_STOPPED     = 0x02,
    MODE_PREOP       = 0x80,
    MODE_OPERATIONAL = 0x01
};

// --- Function Prototypes ---
void CommonConfig();
void WriteDAC(uint8_t channel, uint16_t value);
void SetupPCNT();
float ReadSpeedKMH(unsigned long prev_time);
void StoppedMode();
void PreOpMode();
void OperationalMode();
void SpeedControl(float speed_kmh);
void ThrottleControl(float speed_kmh);
void AutoStopChallenge(float speed_kmh);
void TractionChallenge(float speed_kmh);
void EnergyRecoveryChallenge(float speed_kmh);

#endif // MOTOR_PCB_H
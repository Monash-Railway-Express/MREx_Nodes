/**
 * @file MREx.h
 * @brief Common header file for shared constants.
 *
 * @details
 * Includes MREx CAN registry object dictionary identifiers, indices, subindices, sizes, interpretations and PDO maps.
 *
 * @date 05/05/2026
 *
 * @version 0.0.0
 *
 * @organisation MREX
 *
 * @see https://docs.google.com/spreadsheets/d/1OaXG5B06xnvpNkGQIkrtbM_n-pCCqvnd99yezD7YYoQ/edit?usp=sharing
 */

#ifndef MREX_H
#define MREX_H

#include <Arduino.h>
#include <CAN_MREx.h>

// Node IDs
#define MOTOR_ID 0x01
#define BRAKES_ID 0x02
#define DRIVER_ID 0x03
#define LIGHTS_ID 0x04
#define AUDIO_ID 0x05
#define AUTOSTOP_ID 0x06
#define BATTERY_ID 0x07
#define LOGGER_ID 0x08
#define LCD_ID 0x09

// Object Dictionaries
uint16_t od_regen_brake;
#define OD_REGEN_BRAKE_INDEX 0x3012
#define OD_REGEN_BRAKE_SUBINDEX 0x00

enum ODServiceBrake : uint8_t {
    NOT_BRAKING         = 1,
    BRAKING             = 0
};

uint8_t od_service_brake_mc;
#define OD_SERVICE_BRAKE_MC_INDEX 0x3012
#define OD_SERVICE_BRAKE_MC_SUBINDEX 0x01
#define ODServiceBrakeMC ODServiceBrake

uint8_t od_service_brake_dc;
#define OD_SERVICE_BRAKE_DC_INDEX 0x3012
#define OD_SERVICE_BRAKE_DC_SUBINDEX 0x02
#define ODServiceBrakeDC ODServiceBrake

// missing size and signed interpretation in registry
#define OD_BRAKE_THRESHOLD_INDEX 0x3012
#define OD_BRAKE_THRESHOLD_SUBINDEX 0x03

uint16_t od_motor_command;
#define OD_MOTOR_COMMAND_INDEX 0x606A
#define OD_MOTOR_COMMAND_SUBINDEX 0x00

uint32_t od_true_speed;
#define OD_TRUE_SPEED_INDEX 0x606C
#define OD_TRUE_SPEED_SUBINDEX 0x00

// missing interpretation in registry
#define OD_TRACTIVE_EFFORT_INDEX 0x606E
#define OD_TRACTIVE_EFFORT_SUBINDEX 0x00

float_t od_kp_1;
#define OD_KP_1_INDEX 0x60F6
#define OD_KP_1_SUBINDEX 0x00

float_t od_ki_1;
#define OD_KI_1_INDEX 0x60F6
#define OD_KI_1_SUBINDEX 0x01

float_t od_kd_1;
#define OD_KD_1_INDEX 0x60F6
#define OD_KD_1_SUBINDEX 0x02

float_t od_kp_2;
#define OD_KP_2_INDEX 0x60F6
#define OD_KP_2_SUBINDEX 0x03

float_t od_ki_2;
#define OD_KI_2_INDEX 0x60F6
#define OD_KI_2_SUBINDEX 0x04

float_t od_kd_2;
#define OD_KD_2_INDEX 0x60F6
#define OD_KD_2_SUBINDEX 0x05

float_t od_kp_3;
#define OD_KP_3_INDEX 0x60F6
#define OD_KP_3_SUBINDEX 0x06

float_t od_ki_3;
#define OD_KI_3_INDEX 0x60F6
#define OD_KI_3_SUBINDEX 0x07

float_t od_kd_3;
#define OD_KD_3_INDEX 0x60F6
#define OD_KD_3_SUBINDEX 0x08

float_t od_kp_4;
#define OD_KP_4_INDEX 0x60F6
#define OD_KP_4_SUBINDEX 0x09

float_t od_ki_4;
#define OD_KI_4_INDEX 0x60F6
#define OD_KI_4_SUBINDEX 0x0A

float_t od_kd_4;
#define OD_KD_4_INDEX 0x60F6
#define OD_KD_4_SUBINDEX 0x0B

float_t od_kp_5;
#define OD_KP_5_INDEX 0x60F6
#define OD_KP_5_SUBINDEX 0x0C

float_t od_ki_5;
#define OD_KI_5_INDEX 0x60F6
#define OD_KI_5_SUBINDEX 0x0D

float_t od_kd_5;
#define OD_KD_5_INDEX 0x60F6
#define OD_KD_5_SUBINDEX 0x0E

// for NMTs, no index and subindex in registry
uint8_t od_operation_mode;
enum ODOperatingMode : uint8_t {
    MODE_STOPPED        = 0x02,
    MODE_PREOP          = 0x80,
    MODE_OPERATIONAL    = 0x01
};

uint8_t od_direction_mode;
#define OD_DIRECTION_MODE_INDEX 0x6060
#define OD_DIRECTION_MODE_SUBINDEX 0x00
enum ODDirectionMode : uint8_t {
    REVERSE_MODE        = 1,
    NEUTRAL_MODE        = 2,
    FORWARD_MODE        = 3
};

uint8_t od_condition_mode;
#define OD_CONDITION_MODE_INDEX 0x6061
#define OD_CONDITION_MODE_SUBINDEX 0x00

uint8_t od_challenge_mode;
#define OD_CHALLENGE_MODE_INDEX 0x6062
#define OD_CHALLENGE_MODE_SUBINDEX 0x00
enum ODChallengeMode : uint8_t {
    CHALLENGE_THROTTLE          = 1,
    CHALLENGE_SPEED_CONTROL     = 2,
    CHALLENGE_AUTO_STOP         = 3,
    CHALLENGE_ENERGY_RECOVERY   = 4,
    CHALLENGE_TRACTION          = 5
};

uint8_t od_horn_toggle;
#define OD_HORN_TOGGLE_INDEX 0x6065
#define OD_HORN_TOGGLE_SUBINDEX 0x00
enum ODHornToggle : uint8_t {
    PLAY_SOUND      = 1,
    HORN_DEFAULT    = 0
};

uint16_t od_temperature_front;
#define OD_TEMPERATURE_FRONT_INDEX 0x1004
#define OD_TEMPERATURE_FRONT_SUBINDEX 0x00

uint16_t od_temperature_rear;
#define OD_TEMPERATURE_REAR_INDEX 0x1004
#define OD_TEMPERATURE_REAR_SUBINDEX 0x01

uint8_t od_autostop_detection;
#define OD_AUTOSTOP_DETECTION_INDEX 0x1050
#define OD_AUTOSTOP_DETECTION_SUBINDEX 0x00

uint32_t od_current;
#define OD_CURRENT_INDEX 0x2000
#define OD_CURRENT_SUBINDEX 0x00

uint16_t od_voltage;
#define OD_VOLTAGE_INDEX 0x2000
#define OD_VOLTAGE_SUBINDEX 0x01

uint16_t od_soc;
#define OD_SOC_INDEX 0x2000
#define OD_SOC_SUBINDEX 0x02

uint32_t od_power;
#define OD_POWER_INDEX 0x2000
#define OD_POWER_SUBINDEX 0x03

uint32_t od_recovered_energy;
#define OD_RECOVERED_ENERGY_INDEX 0x2000
#define OD_RECOVERED_ENERGY_SUBINDEX 0x04

// PDO Maps
#define PDO_TS_COB_ID 0x180 + MOTOR_ID
PdoMapEntry PDO_TS_MAP[] = {
    {OD_TRUE_SPEED_INDEX, OD_TRUE_SPEED_SUBINDEX, 8*sizeof(od_true_speed)}
};

#define PDO_RGB_MTCM_COB_ID 0x180 + DRIVER_ID
PdoMapEntry PDO_RGB_MTCM_MAP[] = {
    {OD_REGEN_BRAKE_INDEX, OD_REGEN_BRAKE_SUBINDEX, 8*sizeof(od_regen_brake)},
    {OD_MOTOR_COMMAND_INDEX, OD_MOTOR_COMMAND_SUBINDEX, 8*sizeof(od_motor_command)}
};

#endif // MREX_H
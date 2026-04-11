#ifndef DRIVER_SCREEN_NODE_H
#define DRIVER_SCREEN_NODE_H

#include <Arduino.h>

/**
 * @file driver_screen_node.h
 * @brief Public interface and configuration for the MREx driver screen node.
 *
 * @details
 * This header stores the top-level configuration for the driver screen node,
 * including pin definitions, CAN/Object Dictionary constants, timing
 * constants, and public function prototypes used by the Arduino top module.
 *
 * @author Aditya Dinesh Kumar
 *
 * @date 08/04/2026
 *
 * @version 1.0.1
 *
 * @organisation MREX
 *
 * @see LCDscreen.ino
 */

#define TX_GPIO_NUM GPIO_NUM_41
#define RX_GPIO_NUM GPIO_NUM_42

static const uint8_t NODE_ID = 0x09U;

static const uint16_t SCREEN_W = 800U;
static const uint16_t SCREEN_H = 480U;
static const uint16_t DISPLAY_BUFFER_ROWS = 40U;

static const uint16_t ADC_MAX_VALUE = 1023U;

static const uint32_t SERIAL_BAUD_RATE = 115200UL;
static const uint32_t STARTUP_DELAY_MS = 1000UL;
static const uint32_t LV_HANDLER_PERIOD_MS = 5UL;

static const uint8_t RPDO_INDEX = 0U;
static const uint16_t RPDO_COB_ID = 0x183U;
static const uint16_t RPDO_INHIBIT_TIME = 255U;
static const uint16_t RPDO_EVENT_TIMER_MS = 100U;
static const uint8_t RPDO_ENTRY_COUNT = 4U;

static const uint16_t PDO_MAP_SIZE_U16_BITS = 16U;
static const uint16_t PDO_MAP_SIZE_U8_BITS = 8U;

static const uint16_t OD_INDEX_DESIRED_SPEED = 0x60FFU;
static const uint8_t OD_SUBINDEX_DESIRED_SPEED = 0x00U;

static const uint16_t OD_INDEX_REGEN_BRAKE = 0x3012U;
static const uint8_t OD_SUBINDEX_REGEN_BRAKE = 0x00U;

static const uint16_t OD_INDEX_DIRECTION_MODE = 0x6060U;
static const uint8_t OD_SUBINDEX_DIRECTION_MODE = 0x00U;

static const uint16_t OD_INDEX_SERVICE_BRAKE_STATE = 0x3013U;
static const uint8_t OD_SUBINDEX_SERVICE_BRAKE_STATE = 0x00U;

static const uint8_t OD_ACCESS_RW = 2U;

static const size_t STATUS_TEXT_BUFFER_SIZE = 32U;
static const size_t ALERT_LINE_BUFFER_SIZE = 48U;

/**
 * @brief Operating modes received from CAN MREx NMT state decoding.
 */
enum OperatingMode : uint8_t {
  MODE_STOPPED = 0x02U,
  MODE_PREOP = 0x80U,
  MODE_OPERATIONAL = 0x01U
};

/**
 * @brief Arduino setup entry point for the driver screen node.
 */
void setup(void);

/**
 * @brief Arduino loop entry point for the driver screen node.
 */
void loop(void);

/**
 * @brief Run screen-node behaviour while the node is in Stopped mode.
 */
void StoppedMode(void);

/**
 * @brief Run screen-node behaviour while the node is in Pre-Operational mode.
 */
void PreOpMode(void);

/**
 * @brief Run screen-node behaviour while the node is in Operational mode.
 */
void OperationalMode(void);

#endif // DRIVER_SCREEN_NODE_H

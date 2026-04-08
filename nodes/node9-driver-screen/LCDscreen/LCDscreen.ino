/**
 * @file LCDscreen.ino
 * @brief MREx driver screen node with LVGL-based minimal user interface.
 *
 * @details
 * This top-level Arduino module receives Object Dictionary values from the
 * driver controls node over CAN MREx, mirrors those values into display-safe
 * buffers, and renders the resulting UI on the TFT screen using LVGL. The code
 * is split so that CAN handling, display updates, and UI construction stay
 * separate and maintainable while preserving existing behaviour.
 *
 * @author Aditya Dinesh Kumar
 *
 * @date 08/04/2026
 *
 * @version 1.0.1
 *
 * @organisation MREX
 *
 * @see driver_screen_node.h
 */

#include "driver_screen_node.h"

#include <Arduino.h>
#include <CAN_MREx.h>
#include <TFT_eSPI.h>

#define LV_CONF_INCLUDE_SIMPLE 1
#include "lv_conf.h"
#include <lvgl.h>

// -----------------------------------------------------------------------------
// UI configuration constants
// -----------------------------------------------------------------------------
static const uint16_t SPEED_ARC_SIZE = 310U;
static const uint16_t BRAKE_ARC_SIZE = 220U;
static const uint16_t ARC_TRACK_WIDTH = 11U;
static const uint16_t ARC_ROTATION_DEG = 135U;
static const uint16_t ARC_SWEEP_DEG = 270U;

static const int16_t TITLE_X_OFFSET = 16;
static const int16_t TITLE_Y_OFFSET = 8;

static const int16_t SCREEN_CENTER_Y = 250;
static const int16_t SPEED_TITLE_X_OFFSET = -32;
static const int16_t SPEED_TITLE_Y_OFFSET = -210;
static const int16_t BRAKE_TITLE_Y_OFFSET = 8;
static const int16_t VALUE_LABEL_GAP_Y = 6;

static const int16_t TOP_PANEL_Y = 130;
static const int16_t PANEL_BOX_W = 180;
static const int16_t PANEL_BOX_H = 90;
static const int16_t PANEL_GAP_Y = 14;
static const int16_t LEFT_PANEL_X = 12;
static const int16_t RIGHT_PANEL_MARGIN = 12;

static const int16_t ALERTS_BOX_H = 140;
static const int16_t ALERT_MAJOR_Y = 24;
static const int16_t ALERT_MINOR_Y = 48;
static const int16_t STATUS_LABEL_Y_OFFSET = 10;

static const uint8_t PANEL_RADIUS = 10U;
static const uint8_t PANEL_BORDER_WIDTH = 2U;
static const uint8_t PANEL_PADDING = 10U;

static const lv_opa_t PANEL_BG_OPACITY = LV_OPA_30;
static const lv_opa_t PANEL_BORDER_OPACITY = LV_OPA_40;
static const lv_opa_t TRACK_OPACITY = LV_OPA_70;
static const lv_opa_t SCREEN_BG_OPACITY = LV_OPA_100;

static const lv_color_t PANEL_BG_COLOR = lv_color_make(0U, 0U, 0U);
static const lv_color_t PANEL_BORDER_COLOR = lv_color_make(200U, 200U, 200U);
static const lv_color_t SCREEN_BG_TOP_COLOR = lv_color_make(18U, 0U, 35U);
static const lv_color_t SCREEN_BG_BOTTOM_COLOR = lv_color_make(90U, 20U, 140U);
static const lv_color_t ARC_TRACK_COLOR = lv_color_make(140U, 120U, 170U);
static const lv_color_t STATUS_TEXT_COLOR = lv_color_make(210U, 230U, 255U);
static const lv_color_t MODE_TEXT_COLOR = lv_color_make(210U, 255U, 210U);

// These custom colours preserve the existing hardware appearance on the panel.
static const lv_color_t MAJOR_ALERT_COLOR = lv_color_make(0U, 0U, 255U);
static const lv_color_t MINOR_ALERT_COLOR = lv_color_make(0U, 128U, 255U);

// -----------------------------------------------------------------------------
// Global node state
// -----------------------------------------------------------------------------
static uint8_t node_id = NODE_ID;

static TFT_eSPI tft;
static lv_display_t *display_handle = NULL;
static lv_color_t display_buf_1[SCREEN_W * DISPLAY_BUFFER_ROWS];
static lv_color_t display_buf_2[SCREEN_W * DISPLAY_BUFFER_ROWS];

static lv_obj_t *speed_arc = NULL;
static lv_obj_t *brake_arc = NULL;
static lv_obj_t *brake_title_lbl = NULL;
static lv_obj_t *brake_value_lbl = NULL;
static lv_obj_t *speed_title_lbl = NULL;
static lv_obj_t *speed_value_lbl = NULL;
static lv_obj_t *mode_box = NULL;
static lv_obj_t *alerts_box = NULL;
static lv_obj_t *brake_status_box = NULL;
static lv_obj_t *direction_box = NULL;
static lv_obj_t *mode_title_lbl = NULL;
static lv_obj_t *mode_value_lbl = NULL;
static lv_obj_t *alerts_title_lbl = NULL;
static lv_obj_t *major_alert_lbl = NULL;
static lv_obj_t *minor_alert_lbl = NULL;
static lv_obj_t *brake_status_title_lbl = NULL;
static lv_obj_t *brake_status_value_lbl = NULL;
static lv_obj_t *direction_title_lbl = NULL;
static lv_obj_t *direction_value_lbl = NULL;

// OD 0x60FF:00 - Desired speed command from driver controls. Units: ADC counts.
// Direction: read/write. Mapped from RPDO0.
static uint16_t od_desired_speed = 0U;

// OD 0x3012:00 - Regen brake request from driver controls. Units: ADC counts.
// Direction: read/write. Mapped from RPDO0.
static uint16_t od_regen_brake = 0U;

// OD 0x6060:00 - Direction mode selection. Units: enum value.
// Direction: read/write. Mapped from RPDO0.
static uint8_t od_direction_mode = 2U;

// OD 0x3013:00 - Service brake state. Units: boolean-like state.
// Direction: read/write. Mapped from RPDO0.
static uint8_t od_service_brake_state = 0U;

static uint16_t rx_speed_adc = 0U;
static uint16_t rx_brake_adc = 0U;
static bool rx_brakes_applied = false;
static char rx_major_alert[STATUS_TEXT_BUFFER_SIZE] = "None";
static char rx_minor_alert[STATUS_TEXT_BUFFER_SIZE] = "None";
static char rx_mode_str[STATUS_TEXT_BUFFER_SIZE] = "Stopped";
static char rx_direction_str[STATUS_TEXT_BUFFER_SIZE] = "Neutral";
static bool display_data_updated = false;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------
static void _FlushDisplay(lv_display_t *display, const lv_area_t *area, uint8_t *pixelMap)
{
  uint32_t width = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  uint32_t height = static_cast<uint32_t>(area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, width, height);
  tft.pushColors(reinterpret_cast<uint16_t *>(pixelMap), width * height, true);
  tft.endWrite();

  lv_display_flush_ready(display);
}

static void _StylePanel(lv_obj_t *obj)
{
  lv_obj_set_style_bg_color(obj, PANEL_BG_COLOR, 0);
  lv_obj_set_style_bg_opa(obj, PANEL_BG_OPACITY, 0);
  lv_obj_set_style_border_color(obj, PANEL_BORDER_COLOR, 0);
  lv_obj_set_style_border_opa(obj, PANEL_BORDER_OPACITY, 0);
  lv_obj_set_style_border_width(obj, PANEL_BORDER_WIDTH, 0);
  lv_obj_set_style_radius(obj, PANEL_RADIUS, 0);
  lv_obj_set_style_pad_all(obj, PANEL_PADDING, 0);
}

static void _ApplyBackground(lv_obj_t *screen)
{
  lv_obj_set_style_bg_opa(screen, SCREEN_BG_OPACITY, 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_color(screen, SCREEN_BG_TOP_COLOR, 0);
  lv_obj_set_style_bg_grad_color(screen, SCREEN_BG_BOTTOM_COLOR, 0);
}

static void _CreateTitle(lv_obj_t *screen)
{
  lv_obj_t *title = lv_label_create(screen);

  lv_label_set_text(title, "MREx");
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, TITLE_X_OFFSET, TITLE_Y_OFFSET);
}

static void _CreateGaugeArcs(lv_obj_t *screen)
{
  int16_t centerX = static_cast<int16_t>(SCREEN_W / 2U);

  speed_arc = lv_arc_create(screen);
  lv_obj_set_size(speed_arc, SPEED_ARC_SIZE, SPEED_ARC_SIZE);
  lv_obj_set_pos(
    speed_arc,
    centerX - static_cast<int16_t>(SPEED_ARC_SIZE / 2U),
    SCREEN_CENTER_Y - static_cast<int16_t>(SPEED_ARC_SIZE / 2U)
  );
  lv_arc_set_rotation(speed_arc, ARC_ROTATION_DEG);
  lv_arc_set_bg_angles(speed_arc, 0, ARC_SWEEP_DEG);
  lv_arc_set_range(speed_arc, 0, ADC_MAX_VALUE);
  lv_arc_set_value(speed_arc, 0);
  lv_obj_remove_style(speed_arc, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(speed_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(speed_arc, ARC_TRACK_WIDTH, LV_PART_MAIN);
  lv_obj_set_style_arc_width(speed_arc, ARC_TRACK_WIDTH, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(speed_arc, ARC_TRACK_COLOR, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(speed_arc, TRACK_OPACITY, LV_PART_MAIN);
  lv_obj_set_style_arc_color(speed_arc, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);

  brake_arc = lv_arc_create(screen);
  lv_obj_set_size(brake_arc, BRAKE_ARC_SIZE, BRAKE_ARC_SIZE);
  lv_obj_set_pos(
    brake_arc,
    centerX - static_cast<int16_t>(BRAKE_ARC_SIZE / 2U),
    SCREEN_CENTER_Y - static_cast<int16_t>(BRAKE_ARC_SIZE / 2U)
  );
  lv_arc_set_rotation(brake_arc, ARC_ROTATION_DEG);
  lv_arc_set_bg_angles(brake_arc, 0, ARC_SWEEP_DEG);
  lv_arc_set_range(brake_arc, 0, ADC_MAX_VALUE);
  lv_arc_set_value(brake_arc, 0);
  lv_obj_remove_style(brake_arc, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(brake_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(brake_arc, ARC_TRACK_WIDTH, LV_PART_MAIN);
  lv_obj_set_style_arc_width(brake_arc, ARC_TRACK_WIDTH, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(brake_arc, ARC_TRACK_COLOR, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(brake_arc, TRACK_OPACITY, LV_PART_MAIN);
  lv_obj_set_style_arc_color(brake_arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);

  brake_title_lbl = lv_label_create(screen);
  lv_label_set_text(brake_title_lbl, "Brake");
  lv_obj_set_style_text_color(brake_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(brake_title_lbl, &lv_font_montserrat_20, 0);
  lv_obj_align(brake_title_lbl, LV_ALIGN_CENTER, 0, BRAKE_TITLE_Y_OFFSET);

  brake_value_lbl = lv_label_create(screen);
  lv_label_set_text(brake_value_lbl, "0");
  lv_obj_set_style_text_color(brake_value_lbl, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_set_style_text_font(brake_value_lbl, &lv_font_montserrat_22, 0);
  lv_obj_align_to(brake_value_lbl, brake_title_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, VALUE_LABEL_GAP_Y);

  speed_title_lbl = lv_label_create(screen);
  lv_label_set_text(speed_title_lbl, "Speed");
  lv_obj_set_style_text_color(speed_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(speed_title_lbl, &lv_font_montserrat_20, 0);
  lv_obj_set_pos(speed_title_lbl, centerX + SPEED_TITLE_X_OFFSET, SCREEN_CENTER_Y + SPEED_TITLE_Y_OFFSET);

  speed_value_lbl = lv_label_create(screen);
  lv_label_set_text(speed_value_lbl, "0");
  lv_obj_set_style_text_color(speed_value_lbl, lv_palette_main(LV_PALETTE_CYAN), 0);
  lv_obj_set_style_text_font(speed_value_lbl, &lv_font_montserrat_22, 0);
  lv_obj_align_to(speed_value_lbl, speed_title_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, VALUE_LABEL_GAP_Y);
}

static void _CreateLeftPanels(lv_obj_t *screen)
{
  alerts_box = lv_obj_create(screen);
  lv_obj_set_size(alerts_box, PANEL_BOX_W, ALERTS_BOX_H);
  lv_obj_set_pos(alerts_box, LEFT_PANEL_X, TOP_PANEL_Y);
  _StylePanel(alerts_box);

  alerts_title_lbl = lv_label_create(alerts_box);
  lv_label_set_text(alerts_title_lbl, "Emergency");
  lv_obj_set_style_text_color(alerts_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(alerts_title_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align(alerts_title_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

  major_alert_lbl = lv_label_create(alerts_box);
  lv_label_set_text(major_alert_lbl, "Major: None");
  lv_obj_set_style_text_color(major_alert_lbl, MAJOR_ALERT_COLOR, 0);
  lv_obj_set_style_text_font(major_alert_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(major_alert_lbl, 0, ALERT_MAJOR_Y);

  minor_alert_lbl = lv_label_create(alerts_box);
  lv_label_set_text(minor_alert_lbl, "Minor: None");
  lv_obj_set_style_text_color(minor_alert_lbl, MINOR_ALERT_COLOR, 0);
  lv_obj_set_style_text_font(minor_alert_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(minor_alert_lbl, 0, ALERT_MINOR_Y);

  brake_status_box = lv_obj_create(screen);
  lv_obj_set_size(brake_status_box, PANEL_BOX_W, PANEL_BOX_H);
  lv_obj_set_pos(brake_status_box, LEFT_PANEL_X, TOP_PANEL_Y + ALERTS_BOX_H + PANEL_GAP_Y);
  _StylePanel(brake_status_box);

  brake_status_title_lbl = lv_label_create(brake_status_box);
  lv_label_set_text(brake_status_title_lbl, "Service Brakes");
  lv_obj_set_style_text_color(brake_status_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(brake_status_title_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align(brake_status_title_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

  brake_status_value_lbl = lv_label_create(brake_status_box);
  lv_label_set_text(brake_status_value_lbl, "Not Applied");
  lv_obj_set_style_text_color(brake_status_value_lbl, STATUS_TEXT_COLOR, 0);
  lv_obj_set_style_text_font(brake_status_value_lbl, &lv_font_montserrat_22, 0);
  lv_obj_align(brake_status_value_lbl, LV_ALIGN_CENTER, 0, STATUS_LABEL_Y_OFFSET);
}

static void _CreateRightPanels(lv_obj_t *screen)
{
  int16_t rightPanelX = static_cast<int16_t>(SCREEN_W) - PANEL_BOX_W - RIGHT_PANEL_MARGIN;

  mode_box = lv_obj_create(screen);
  lv_obj_set_size(mode_box, PANEL_BOX_W, PANEL_BOX_H);
  lv_obj_set_pos(mode_box, rightPanelX, TOP_PANEL_Y);
  _StylePanel(mode_box);

  mode_title_lbl = lv_label_create(mode_box);
  lv_label_set_text(mode_title_lbl, "Mode");
  lv_obj_set_style_text_color(mode_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(mode_title_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align(mode_title_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

  mode_value_lbl = lv_label_create(mode_box);
  lv_label_set_text(mode_value_lbl, "Stopped");
  lv_obj_set_style_text_color(mode_value_lbl, MODE_TEXT_COLOR, 0);
  lv_obj_set_style_text_font(mode_value_lbl, &lv_font_montserrat_24, 0);
  lv_obj_align(mode_value_lbl, LV_ALIGN_CENTER, 0, STATUS_LABEL_Y_OFFSET);

  direction_box = lv_obj_create(screen);
  lv_obj_set_size(direction_box, PANEL_BOX_W, PANEL_BOX_H);
  lv_obj_set_pos(direction_box, rightPanelX, TOP_PANEL_Y + PANEL_BOX_H + PANEL_GAP_Y);
  _StylePanel(direction_box);

  direction_title_lbl = lv_label_create(direction_box);
  lv_label_set_text(direction_title_lbl, "Direction");
  lv_obj_set_style_text_color(direction_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(direction_title_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align(direction_title_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

  direction_value_lbl = lv_label_create(direction_box);
  lv_label_set_text(direction_value_lbl, "Neutral");
  lv_obj_set_style_text_color(direction_value_lbl, STATUS_TEXT_COLOR, 0);
  lv_obj_set_style_text_font(direction_value_lbl, &lv_font_montserrat_24, 0);
  lv_obj_align(direction_value_lbl, LV_ALIGN_CENTER, 0, STATUS_LABEL_Y_OFFSET);
}

static void _CreateGui(void)
{
  lv_obj_t *screen = lv_screen_active();

  _ApplyBackground(screen);
  _CreateTitle(screen);
  _CreateGaugeArcs(screen);
  _CreateLeftPanels(screen);
  _CreateRightPanels(screen);
}

static const char *_GetModeText(uint8_t rawMode)
{
  if (rawMode == MODE_STOPPED) {
    return "Stopped";
  }
  if (rawMode == MODE_PREOP) {
    return "Pre-Op";
  }
  if (rawMode == MODE_OPERATIONAL) {
    return "Operational";
  }

  return "Unknown";
}

static const char *_GetDirectionText(uint8_t rawDirection)
{
  if (rawDirection == 1U) {
    return "Forward";
  }
  if (rawDirection == 2U) {
    return "Neutral";
  }
  if (rawDirection == 3U) {
    return "Reverse";
  }

  return "Unknown";
}

static void _UpdateGauges(uint16_t speedAdc, uint16_t brakeAdc)
{
  char valueText[STATUS_TEXT_BUFFER_SIZE];

  speedAdc = constrain(speedAdc, 0U, ADC_MAX_VALUE);
  brakeAdc = constrain(brakeAdc, 0U, ADC_MAX_VALUE);

  lv_arc_set_value(speed_arc, speedAdc);
  lv_arc_set_value(brake_arc, brakeAdc);

  snprintf(valueText, sizeof(valueText), "%u", brakeAdc);
  lv_label_set_text(brake_value_lbl, valueText);

  snprintf(valueText, sizeof(valueText), "%u", speedAdc);
  lv_label_set_text(speed_value_lbl, valueText);
}

static void _UpdateStatusBoxes(
  const char *modeStr,
  const char *majorAlert,
  const char *minorAlert,
  bool brakesApplied,
  const char *directionStr
)
{
  char alertLine[ALERT_LINE_BUFFER_SIZE];

  lv_label_set_text(mode_value_lbl, modeStr);

  snprintf(alertLine, sizeof(alertLine), "Major: %s", majorAlert);
  lv_label_set_text(major_alert_lbl, alertLine);

  snprintf(alertLine, sizeof(alertLine), "Minor: %s", minorAlert);
  lv_label_set_text(minor_alert_lbl, alertLine);

  lv_label_set_text(brake_status_value_lbl, brakesApplied ? "Applied" : "Not Applied");
  if (brakesApplied) {
    lv_obj_set_style_text_color(brake_status_value_lbl, lv_palette_main(LV_PALETTE_RED), 0);
  } else {
    lv_obj_set_style_text_color(brake_status_value_lbl, STATUS_TEXT_COLOR, 0);
  }

  lv_label_set_text(direction_value_lbl, directionStr);
  if (strcmp(directionStr, "Forward") == 0) {
    lv_obj_set_style_text_color(direction_value_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
  } else if (strcmp(directionStr, "Reverse") == 0) {
    lv_obj_set_style_text_color(direction_value_lbl, lv_palette_main(LV_PALETTE_RED), 0);
  } else {
    lv_obj_set_style_text_color(direction_value_lbl, STATUS_TEXT_COLOR, 0);
  }
}

static void _RefreshDisplay(void)
{
  Serial.print("Speed ADC: ");
  Serial.print(rx_speed_adc);
  Serial.print(" | Brake ADC: ");
  Serial.print(rx_brake_adc);
  Serial.print(" | Mode: ");
  Serial.print(rx_mode_str);
  Serial.print(" | Dir: ");
  Serial.print(rx_direction_str);
  Serial.print(" | ServiceBrake: ");
  Serial.println(rx_brakes_applied ? "Applied" : "Not Applied");

  _UpdateGauges(rx_speed_adc, rx_brake_adc);
  _UpdateStatusBoxes(
    rx_mode_str,
    rx_major_alert,
    rx_minor_alert,
    rx_brakes_applied,
    rx_direction_str
  );
}

static void _CopyCanDataToDisplay(void)
{
  // Mirror OD-backed CAN values into display-only variables so UI code never
  // writes back into communication state by accident.
  rx_speed_adc = od_desired_speed;
  rx_brake_adc = od_regen_brake;
  rx_brakes_applied = (od_service_brake_state != 0U);

  snprintf(rx_mode_str, sizeof(rx_mode_str), "%s", _GetModeText(nodeOperatingMode));
  snprintf(rx_direction_str, sizeof(rx_direction_str), "%s", _GetDirectionText(od_direction_mode));
  snprintf(rx_major_alert, sizeof(rx_major_alert), "%s", "None");
  snprintf(rx_minor_alert, sizeof(rx_minor_alert), "%s", "None");

  display_data_updated = true;
}

static void _InitCanMrex(void)
{
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, node_id);

  xTaskCreatePinnedToCore(
    CAN_Task,
    "CAN Task",
    4096,
    &node_id,
    3,
    NULL,
    0
  );

  registerODEntry(
    OD_INDEX_DESIRED_SPEED,
    OD_SUBINDEX_DESIRED_SPEED,
    OD_ACCESS_RW,
    sizeof(od_desired_speed),
    &od_desired_speed
  );
  registerODEntry(
    OD_INDEX_REGEN_BRAKE,
    OD_SUBINDEX_REGEN_BRAKE,
    OD_ACCESS_RW,
    sizeof(od_regen_brake),
    &od_regen_brake
  );
  registerODEntry(
    OD_INDEX_DIRECTION_MODE,
    OD_SUBINDEX_DIRECTION_MODE,
    OD_ACCESS_RW,
    sizeof(od_direction_mode),
    &od_direction_mode
  );
  registerODEntry(
    OD_INDEX_SERVICE_BRAKE_STATE,
    OD_SUBINDEX_SERVICE_BRAKE_STATE,
    OD_ACCESS_RW,
    sizeof(od_service_brake_state),
    &od_service_brake_state
  );

  configureRPDO(RPDO_INDEX, RPDO_COB_ID, RPDO_INHIBIT_TIME, RPDO_EVENT_TIMER_MS);

  PdoMapEntry rpdoEntries[RPDO_ENTRY_COUNT] = {
    {OD_INDEX_DESIRED_SPEED, OD_SUBINDEX_DESIRED_SPEED, PDO_MAP_SIZE_U16_BITS},
    {OD_INDEX_REGEN_BRAKE, OD_SUBINDEX_REGEN_BRAKE, PDO_MAP_SIZE_U16_BITS},
    {OD_INDEX_DIRECTION_MODE, OD_SUBINDEX_DIRECTION_MODE, PDO_MAP_SIZE_U8_BITS},
    {OD_INDEX_SERVICE_BRAKE_STATE, OD_SUBINDEX_SERVICE_BRAKE_STATE, PDO_MAP_SIZE_U8_BITS}
  };
  mapRPDO(RPDO_INDEX, rpdoEntries, RPDO_ENTRY_COUNT);
}

static void _InitDisplay(void)
{
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  lv_init();

  display_handle = lv_display_create(SCREEN_W, SCREEN_H);
  lv_display_set_flush_cb(display_handle, _FlushDisplay);
  lv_display_set_buffers(
    display_handle,
    display_buf_1,
    display_buf_2,
    sizeof(display_buf_1),
    LV_DISPLAY_RENDER_MODE_PARTIAL
  );

  _CreateGui();
  _RefreshDisplay();
}

static void _RunDisplayCycle(void)
{
  _CopyCanDataToDisplay();

  if (display_data_updated) {
    display_data_updated = false;
    _RefreshDisplay();
  }
}

void StoppedMode(void)
{
  _RunDisplayCycle();
}

void PreOpMode(void)
{
  _RunDisplayCycle();
}

void OperationalMode(void)
{
  _RunDisplayCycle();
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(STARTUP_DELAY_MS);
  Serial.println("Serial Coms started at 115200 baud");

  _InitCanMrex();
  _InitDisplay();
}

void loop(void)
{
  static uint32_t tickMsPrev = 0UL;
  static uint32_t lvHandlerMsPrev = 0UL;
  static uint8_t operatingModePrev = 0xFFU;

  uint32_t nowMs = millis();
  uint8_t rawOperatingMode = nodeOperatingMode;
  OperatingMode mode = static_cast<OperatingMode>(rawOperatingMode);

  if (tickMsPrev == 0UL) {
    tickMsPrev = nowMs;
  }

  lv_tick_inc(nowMs - tickMsPrev);
  tickMsPrev = nowMs;

  switch (mode) {
    case MODE_STOPPED:
      StoppedMode();
      break;

    case MODE_PREOP:
      PreOpMode();
      break;

    case MODE_OPERATIONAL:
      OperationalMode();
      break;

    default:
      if (rawOperatingMode != operatingModePrev) {
        Serial.print("Warning: Unknown operating mode received: 0x");
        Serial.println(rawOperatingMode, HEX);
      }
      StoppedMode();
      break;
  }

  operatingModePrev = rawOperatingMode;

  // LVGL is serviced periodically without blocking the rest of the node loop.
  if ((nowMs - lvHandlerMsPrev) >= LV_HANDLER_PERIOD_MS) {
    lv_timer_handler();
    lvHandlerMsPrev = nowMs;
  }
}
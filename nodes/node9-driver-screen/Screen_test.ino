/**
 * CAN MREX Driver Screen Node 
 * Screen node (Node 9) receives data from Driver Controls node (Node 3)
 *
 * IMPORTANT:
 * I've left out the below parts coz I'm a lil unsure still:
 *
 * 1. OD ENTRIES 
 * 2. RPDO REGISTRATION  
 * 3. RECEIVED CAN DATA INTO DISPLAY VARIABLES  
 */

#include <Arduino.h>
#include <can_mrex.h>
#include <TFT_eSPI.h>

#define LV_CONF_INCLUDE_SIMPLE 1
#include "lv_conf.h"
#include <lvgl.h>

// -----------------------------------------------------------------------------
// CAN setup
// -----------------------------------------------------------------------------
const uint8_t nodeID = 9;   // Screen node

#define TX_GPIO_NUM GPIO_NUM_1
#define RX_GPIO_NUM GPIO_NUM_2

// -----------------------------------------------------------------------------
// TFT / LVGL setup (screen visual library)
// -----------------------------------------------------------------------------
static const uint16_t SCREEN_W = 800;
static const uint16_t SCREEN_H = 480;

#define TFT_SWAP_BYTES 1

TFT_eSPI tft;
static lv_display_t *disp;
static lv_color_t buf1[SCREEN_W * 40];
static lv_color_t buf2[SCREEN_W * 40];

// -----------------------------------------------------------------------------
// UI stuff
// -----------------------------------------------------------------------------
static lv_obj_t *speed_arc, *brake_arc;
static lv_obj_t *brake_title_lbl, *brake_value_lbl;
static lv_obj_t *speed_title_lbl, *speed_value_lbl;

static lv_obj_t *alert_box, *mode_box;
static lv_obj_t *alert_major_lbl, *alert_minor_lbl;
static lv_obj_t *battery_lbl1, *battery_lbl2;
static lv_obj_t *mode_lbl;

static lv_obj_t *fuel_box, *regen_box, *brake_status_box;
static lv_obj_t *fuel_title_lbl, *fuel_value_lbl;
static lv_obj_t *regen_title_lbl, *regen_value_lbl;
static lv_obj_t *brake_status_title_lbl, *brake_status_value_lbl;

static lv_obj_t *challenge_box;
static lv_obj_t *ch_title_lbl;
static lv_obj_t *ch1_name_lbl, *ch1_status_lbl;
static lv_obj_t *ch2_name_lbl, *ch2_status_lbl;
static lv_obj_t *ch3_name_lbl, *ch3_status_lbl;

// -----------------------------------------------------------------------------
// Display variables
// These get shown on the screen
// -----------------------------------------------------------------------------
float rx_speed_kmh = 0.0f;
int   rx_brake_pct = 0;

float rx_packV = 0.0f;
float rx_packA = 0.0f;
float rx_soc = 0.0f;
float rx_tempC = 0.0f;

float rx_powerCapPct = 100.0f;
float rx_recoveredWh = 0.0f;
bool  rx_brakesApplied = false;

uint8_t rx_activeChallengeMode = 1;  // 1..3

char rx_majorAlert[32] = "None";
char rx_minorAlert[32] = "None";
char rx_modeStr[32]    = "Stopped";

bool displayDataUpdated = false;

// -----------------------------------------------------------------------------
// ===== DECLARE OD VARIABLES HERE =====
// These are placeholders for now, gotta replace with the actual ones 
// -----------------------------------------------------------------------------

uint16_t odDesiredSpeed = 0;
uint16_t odRegenBrake = 0;
// uint8_t odMode = 0;
// uint8_t odChallenge = 0;
// uint16_t odPackV = 0;
// int16_t odPackA = 0;
// uint8_t odSOC = 0;
// int16_t odTemp = 0;
// uint16_t odPowerCap = 100;
// uint16_t odRecoveredWh = 0;
// uint8_t odBrakeStatus = 0;
// uint8_t odMajorAlert = 0;
// uint8_t odMinorAlert = 0;

// -----------------------------------------------------------------------------
// LVGL display flush
// -----------------------------------------------------------------------------
static void my_disp_flush(lv_display_t *d, const lv_area_t *area, uint8_t *px_map)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)px_map, w * h, TFT_SWAP_BYTES);
  tft.endWrite();

  lv_display_flush_ready(d);
}

// -----------------------------------------------------------------------------
// UI helpers
// -----------------------------------------------------------------------------
static void style_panel(lv_obj_t *obj)
{
  lv_obj_set_style_bg_color(obj, lv_color_make(0, 0, 0), 0);
  lv_obj_set_style_bg_opa(obj, LV_OPA_30, 0);
  lv_obj_set_style_border_color(obj, lv_color_make(200, 200, 200), 0);
  lv_obj_set_style_border_opa(obj, LV_OPA_40, 0);
  lv_obj_set_style_border_width(obj, 2, 0);
  lv_obj_set_style_radius(obj, 10, 0);
  lv_obj_set_style_pad_all(obj, 10, 0);
}

static void set_challenge_status(lv_obj_t *lbl, bool active)
{
  lv_label_set_text(lbl, active ? "Active" : "Inactive");

  if (active) {
    lv_obj_set_style_text_color(lbl, lv_color_make(0, 255, 0), 0);
  } else {
    lv_obj_set_style_text_color(lbl, lv_color_make(0, 0, 255), 0);
  }
}

// -----------------------------------------------------------------------------
// GUI creation (based on style discussed)
// -----------------------------------------------------------------------------
static void create_gui()
{
  lv_obj_t *scr = lv_screen_active();

  lv_obj_set_style_bg_opa(scr, LV_OPA_100, 0);
  lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_color(scr, lv_color_make(18, 0, 35), 0);
  lv_obj_set_style_bg_grad_color(scr, lv_color_make(90, 20, 140), 0);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "MREx");
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 16, 8);

  const int cx = SCREEN_W / 2;
  const int cy = 235;
  lv_color_t track_col = lv_color_make(140, 120, 170);

  speed_arc = lv_arc_create(scr);
  lv_obj_set_size(speed_arc, 310, 310);
  lv_obj_set_pos(speed_arc, cx - 155, cy - 155);
  lv_arc_set_rotation(speed_arc, 135);
  lv_arc_set_bg_angles(speed_arc, 0, 270);
  lv_arc_set_range(speed_arc, 0, 25);
  lv_arc_set_value(speed_arc, 0);
  lv_obj_remove_style(speed_arc, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(speed_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(speed_arc, 11, LV_PART_MAIN);
  lv_obj_set_style_arc_width(speed_arc, 11, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(speed_arc, track_col, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(speed_arc, LV_OPA_70, LV_PART_MAIN);
  lv_obj_set_style_arc_color(speed_arc, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);

  brake_arc = lv_arc_create(scr);
  lv_obj_set_size(brake_arc, 220, 220);
  lv_obj_set_pos(brake_arc, cx - 110, cy - 110);
  lv_arc_set_rotation(brake_arc, 135);
  lv_arc_set_bg_angles(brake_arc, 0, 270);
  lv_arc_set_range(brake_arc, 0, 100);
  lv_arc_set_value(brake_arc, 0);
  lv_obj_remove_style(brake_arc, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(brake_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(brake_arc, 11, LV_PART_MAIN);
  lv_obj_set_style_arc_width(brake_arc, 11, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(brake_arc, track_col, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(brake_arc, LV_OPA_70, LV_PART_MAIN);
  lv_obj_set_style_arc_color(brake_arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);

  brake_title_lbl = lv_label_create(scr);
  lv_label_set_text(brake_title_lbl, "Brake");
  lv_obj_set_style_text_color(brake_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(brake_title_lbl, &lv_font_montserrat_20, 0);
  lv_obj_align(brake_title_lbl, LV_ALIGN_CENTER, 0, -24);

  brake_value_lbl = lv_label_create(scr);
  lv_label_set_text(brake_value_lbl, "0 %");
  lv_obj_set_style_text_color(brake_value_lbl, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_set_style_text_font(brake_value_lbl, &lv_font_montserrat_20, 0);
  lv_obj_align_to(brake_value_lbl, brake_title_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  speed_title_lbl = lv_label_create(scr);
  lv_label_set_text(speed_title_lbl, "Speed");
  lv_obj_set_style_text_color(speed_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(speed_title_lbl, &lv_font_montserrat_20, 0);
  lv_obj_align(speed_title_lbl, LV_ALIGN_TOP_MID, 0, (cy - 155) - 50);

  speed_value_lbl = lv_label_create(scr);
  lv_label_set_text(speed_value_lbl, "0.0 km/h");
  lv_obj_set_style_text_color(speed_value_lbl, lv_palette_main(LV_PALETTE_CYAN), 0);
  lv_obj_set_style_text_font(speed_value_lbl, &lv_font_montserrat_20, 0);
  lv_obj_align_to(speed_value_lbl, speed_title_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  const int box_h = 120;
  const int box_y = SCREEN_H - box_h - 10;
  const int gap   = 10;
  const int left_w  = (SCREEN_W / 2) - (gap / 2) - 10;
  const int right_w = (SCREEN_W / 2) - (gap / 2) - 10;

  alert_box = lv_obj_create(scr);
  lv_obj_set_size(alert_box, left_w, box_h);
  lv_obj_set_pos(alert_box, 10, box_y);
  style_panel(alert_box);

  const int r0 = 0, r1 = 24, r2 = 48, r3 = 72;

  alert_major_lbl = lv_label_create(alert_box);
  lv_label_set_text(alert_major_lbl, "MAJOR: None");
  lv_obj_set_style_text_color(alert_major_lbl, lv_color_make(0, 0, 255), 0);
  lv_obj_set_style_text_font(alert_major_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(alert_major_lbl, 0, r0);

  alert_minor_lbl = lv_label_create(alert_box);
  lv_label_set_text(alert_minor_lbl, "MINOR: None");
  lv_obj_set_style_text_color(alert_minor_lbl, lv_color_make(0, 128, 255), 0);
  lv_obj_set_style_text_font(alert_minor_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(alert_minor_lbl, 0, r1);

  battery_lbl1 = lv_label_create(alert_box);
  lv_label_set_text(battery_lbl1, "Pack: 0.0V  0.0A  0W");
  lv_obj_set_style_text_color(battery_lbl1, lv_color_make(210, 230, 255), 0);
  lv_obj_set_style_text_font(battery_lbl1, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(battery_lbl1, 0, r2);

  battery_lbl2 = lv_label_create(alert_box);
  lv_label_set_text(battery_lbl2, "SOC: 0%  Temp: 0C");
  lv_obj_set_style_text_color(battery_lbl2, lv_color_make(210, 230, 255), 0);
  lv_obj_set_style_text_font(battery_lbl2, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(battery_lbl2, 0, r3);

  mode_box = lv_obj_create(scr);
  lv_obj_set_size(mode_box, right_w, box_h);
  lv_obj_set_pos(mode_box, (SCREEN_W / 2) + (gap / 2), box_y);
  style_panel(mode_box);

  lv_obj_t *mode_title = lv_label_create(mode_box);
  lv_label_set_text(mode_title, "Operation Mode");
  lv_obj_set_style_text_color(mode_title, lv_color_white(), 0);
  lv_obj_set_style_text_font(mode_title, &lv_font_montserrat_18, 0);
  lv_obj_align(mode_title, LV_ALIGN_TOP_LEFT, 0, 0);

  mode_lbl = lv_label_create(mode_box);
  lv_label_set_text(mode_lbl, "Current mode: Stopped");
  lv_obj_set_style_text_color(mode_lbl, lv_color_make(210, 255, 210), 0);
  lv_obj_set_style_text_font(mode_lbl, &lv_font_montserrat_24, 0);
  lv_obj_align(mode_lbl, LV_ALIGN_CENTER, 0, 10);

  const int left_x = 10;
  const int left_col_w = left_w - 150;
  const int top_y = 60;
  const int bottom_limit = box_y - 10;
  const int available_h = bottom_limit - top_y;
  const int left_gap = 10;
  const int left_box_h = (available_h - (2 * left_gap)) / 3;

  fuel_box = lv_obj_create(scr);
  lv_obj_set_size(fuel_box, left_col_w, left_box_h);
  lv_obj_set_pos(fuel_box, left_x, top_y);
  style_panel(fuel_box);

  fuel_title_lbl = lv_label_create(fuel_box);
  lv_label_set_text(fuel_title_lbl, "Fuel/Charge Level");
  lv_obj_set_style_text_color(fuel_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(fuel_title_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(fuel_title_lbl, 0, 0);

  fuel_value_lbl = lv_label_create(fuel_box);
  lv_label_set_text(fuel_value_lbl, "Power cap: 100%");
  lv_obj_set_style_text_color(fuel_value_lbl, lv_color_make(210, 230, 255), 0);
  lv_obj_set_style_text_font(fuel_value_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align(fuel_value_lbl, LV_ALIGN_LEFT_MID, 0, 6);

  regen_box = lv_obj_create(scr);
  lv_obj_set_size(regen_box, left_col_w, left_box_h);
  lv_obj_set_pos(regen_box, left_x, top_y + left_box_h + left_gap);
  style_panel(regen_box);

  regen_title_lbl = lv_label_create(regen_box);
  lv_label_set_text(regen_title_lbl, "Recovered Energy");
  lv_obj_set_style_text_color(regen_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(regen_title_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(regen_title_lbl, 0, 0);

  regen_value_lbl = lv_label_create(regen_box);
  lv_label_set_text(regen_value_lbl, "Recovered: 0.0 Wh");
  lv_obj_set_style_text_color(regen_value_lbl, lv_color_make(210, 230, 255), 0);
  lv_obj_set_style_text_font(regen_value_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align(regen_value_lbl, LV_ALIGN_LEFT_MID, 0, 6);

  brake_status_box = lv_obj_create(scr);
  lv_obj_set_size(brake_status_box, left_col_w, left_box_h);
  lv_obj_set_pos(brake_status_box, left_x, top_y + (2 * (left_box_h + left_gap)));
  style_panel(brake_status_box);

  brake_status_title_lbl = lv_label_create(brake_status_box);
  lv_label_set_text(brake_status_title_lbl, "Brake Status");
  lv_obj_set_style_text_color(brake_status_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(brake_status_title_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(brake_status_title_lbl, 0, 0);

  brake_status_value_lbl = lv_label_create(brake_status_box);
  lv_label_set_text(brake_status_value_lbl, "Brakes Released");
  lv_obj_set_style_text_color(brake_status_value_lbl, lv_color_make(210, 230, 255), 0);
  lv_obj_set_style_text_font(brake_status_value_lbl, &lv_font_montserrat_16, 0);
  lv_obj_align(brake_status_value_lbl, LV_ALIGN_LEFT_MID, 0, 6);

  const int right_margin = 10;
  const int right_x = (cx + 155) + 10;
  const int right_h = (3 * left_box_h) + (2 * left_gap);

  int right_col_w = (SCREEN_W - right_margin) - right_x;
  if (right_col_w < 180) right_col_w = 180;

  challenge_box = lv_obj_create(scr);
  lv_obj_set_size(challenge_box, right_col_w, right_h);
  lv_obj_set_pos(challenge_box, right_x, top_y);
  style_panel(challenge_box);

  ch_title_lbl = lv_label_create(challenge_box);
  lv_label_set_text(ch_title_lbl, "Challenge Modes");
  lv_obj_set_style_text_color(ch_title_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(ch_title_lbl, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(ch_title_lbl, 0, 0);

  const int row_gap = 10;
  const int row_h = (right_h - 28 - (2 * row_gap)) / 3;
  const int start_y = 26;

  ch1_name_lbl = lv_label_create(challenge_box);
  lv_label_set_text(ch1_name_lbl, "Autostop");
  lv_obj_set_style_text_color(ch1_name_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(ch1_name_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(ch1_name_lbl, 0, start_y + 0 * (row_h + row_gap));

  ch1_status_lbl = lv_label_create(challenge_box);
  lv_obj_set_style_text_font(ch1_status_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align_to(ch1_status_lbl, ch1_name_lbl, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

  ch2_name_lbl = lv_label_create(challenge_box);
  lv_label_set_text(ch2_name_lbl, "Traction");
  lv_obj_set_style_text_color(ch2_name_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(ch2_name_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(ch2_name_lbl, 0, start_y + 1 * (row_h + row_gap));

  ch2_status_lbl = lv_label_create(challenge_box);
  lv_obj_set_style_text_font(ch2_status_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align_to(ch2_status_lbl, ch2_name_lbl, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

  ch3_name_lbl = lv_label_create(challenge_box);
  lv_label_set_text(ch3_name_lbl, "Energy Recovery");
  lv_obj_set_style_text_color(ch3_name_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_font(ch3_name_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_pos(ch3_name_lbl, 0, start_y + 2 * (row_h + row_gap));

  ch3_status_lbl = lv_label_create(challenge_box);
  lv_obj_set_style_text_font(ch3_status_lbl, &lv_font_montserrat_18, 0);
  lv_obj_align_to(ch3_status_lbl, ch3_name_lbl, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

  set_challenge_status(ch1_status_lbl, true);
  set_challenge_status(ch2_status_lbl, false);
  set_challenge_status(ch3_status_lbl, false);
}

// -----------------------------------------------------------------------------
// Screen update functions
// -----------------------------------------------------------------------------
static void update_gauges(float speed_kmh, int brake_percent)
{
  if (speed_kmh < 0) speed_kmh = 0;
  if (speed_kmh > 25) speed_kmh = 25;
  brake_percent = constrain(brake_percent, 0, 100);

  lv_arc_set_value(speed_arc, (int)(speed_kmh + 0.5f));
  lv_arc_set_value(brake_arc, brake_percent);

  char txt[32];
  snprintf(txt, sizeof(txt), "%d %%", brake_percent);
  lv_label_set_text(brake_value_lbl, txt);

  snprintf(txt, sizeof(txt), "%.1f km/h", speed_kmh);
  lv_label_set_text(speed_value_lbl, txt);
}

static void update_bottom_panels(
  const char *majorAlert,
  const char *minorAlert,
  float packV, float packA,
  float socPct, float tempC,
  const char *modeStr
) {
  char line[64];

  snprintf(line, sizeof(line), "MAJOR: %s", majorAlert);
  lv_label_set_text(alert_major_lbl, line);

  snprintf(line, sizeof(line), "MINOR: %s", minorAlert);
  lv_label_set_text(alert_minor_lbl, line);

  float watts = packV * packA;
  snprintf(line, sizeof(line), "Pack: %.1fV  %.1fA  %.0fW", packV, packA, watts);
  lv_label_set_text(battery_lbl1, line);

  snprintf(line, sizeof(line), "SOC: %.0f%%  Temp: %.1fC", socPct, tempC);
  lv_label_set_text(battery_lbl2, line);

  snprintf(line, sizeof(line), "Current mode: %s", modeStr);
  lv_label_set_text(mode_lbl, line);
}

static void update_left_panels(float powerCapPct, float recoveredWh, bool brakesApplied)
{
  char line[48];

  snprintf(line, sizeof(line), "Power cap: %.0f%%", powerCapPct);
  lv_label_set_text(fuel_value_lbl, line);

  snprintf(line, sizeof(line), "Recovered: %.1f Wh", recoveredWh);
  lv_label_set_text(regen_value_lbl, line);

  lv_label_set_text(brake_status_value_lbl, brakesApplied ? "Brakes Applied" : "Brakes Released");

  if (brakesApplied) {
    lv_obj_set_style_text_color(brake_status_value_lbl, lv_color_make(0, 0, 255), 0);
  } else {
    lv_obj_set_style_text_color(brake_status_value_lbl, lv_color_make(210, 230, 255), 0);
  }
}

static void update_challenge_modes(uint8_t activeIndex)
{
  set_challenge_status(ch1_status_lbl, activeIndex == 1);
  set_challenge_status(ch2_status_lbl, activeIndex == 2);
  set_challenge_status(ch3_status_lbl, activeIndex == 3);
}

static void refresh_display()
{
  update_gauges(rx_speed_kmh, rx_brake_pct);
  update_bottom_panels(rx_majorAlert, rx_minorAlert, rx_packV, rx_packA, rx_soc, rx_tempC, rx_modeStr);
  update_left_panels(rx_powerCapPct, rx_recoveredWh, rx_brakesApplied);
  update_challenge_modes(rx_activeChallengeMode);
}

// -----------------------------------------------------------------------------
// ===== CAN DATA FROM RELEVANT NODES GO INTO DISPLAY VARIABLES HERE =====
// -----------------------------------------------------------------------------
static void copy_can_data_to_display()
{
  // Examples only — replace these with your real received values

  rx_speed_kmh = (odDesiredSpeed / 1023.0f) * 25.0f;
  rx_brake_pct = (int)((odRegenBrake / 1023.0f) * 100.0f);
  // rx_packV = odPackV / 10.0f;
  // rx_packA = odPackA / 10.0f;
  // rx_soc = odSOC;
  // rx_tempC = odTemp / 10.0f;
  // rx_powerCapPct = odPowerCap;
  // rx_recoveredWh = odRecoveredWh / 10.0f;
  // rx_brakesApplied = odBrakeStatus;
  // rx_activeChallengeMode = odChallenge;

  // strcpy(rx_modeStr, "Stopped");
  // strcpy(rx_majorAlert, "None");
  // strcpy(rx_minorAlert, "None");

  // When new CAN data is received, set this true
  displayDataUpdated = true;
}

// -----------------------------------------------------------------------------
// Setup esp32
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");

  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  // ===========================================================================
  // ====== REGISTER THE OD ENTRIES HERE =====
  // Register all OD entries for the data the screen node will receive
  // ===========================================================================

  // Eg:
  // registerODEntry(...);
    registerODEntry(0x60FF, 0x00, 2, sizeof(odDesiredSpeed), &odDesiredSpeed);
    registerODEntry(0x3012, 0x00, 2, sizeof(odRegenBrake), &odRegenBrake);

    configureRPDO(0, 0x183, 255);

    PdoMapEntry rpdoEntries[] = {
        {0x60FF, 0x00, 16},
        {0x3012, 0x00, 16}
    };

mapRPDO(0, rpdoEntries, 2);
  // ===========================================================================
  // ===== RPDO REGISTRATION HERE =====
  // Register the RPDOs that receive data from Node 3
  // ===========================================================================

  // Eg:
  // registerRPDO(...);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  lv_init();

  disp = lv_display_create(SCREEN_W, SCREEN_H);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

  create_gui();
  refresh_display();
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();

  lv_tick_inc(now - lastTick);
  lastTick = now;

  if (nodeOperatingMode == 0x02) {
    handleCAN(nodeID);
  }

  if (nodeOperatingMode == 0x80) {
    handleCAN(nodeID);
  }

  if (nodeOperatingMode == 0x01) {
    handleCAN(nodeID);
  }

  copy_can_data_to_display();

  if (displayDataUpdated) {
    displayDataUpdated = false;
    refresh_display();
  }

  lv_timer_handler();
  delay(5);
} 
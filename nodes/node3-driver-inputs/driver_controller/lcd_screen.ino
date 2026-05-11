#include <HardwareSerial.h>

HardwareSerial nextionSerial(1);

// ── Previous values cache ──────────────────────────────────────
int prevSpeed        = -1;
int prevThrottle     = -1;
int prevBrake        = -1;
int prevDirection    = -1;
int prevChallenge    = -1;
int prevCondition    = -1;
int prevBrakeStatus  = -1;
uint8_t prevOpMode   = 255;

// ── Timing ─────────────────────────────────────────────────────
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 100;

// ── Nextion helpers ─────────────────────────────────────────────
void sendText(String component, String value) {
  nextionSerial.print(component + ".txt=\"" + value + "\"");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendProgressBar(String component, int value) {
  nextionSerial.print(component + ".val=" + String(value));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendColour(String component, String attr, int colour) {
  nextionSerial.print(component + "." + attr + "=" + String(colour));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void refreshComponent(String component) {
  nextionSerial.print("ref " + component);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

// ── Nextion colour constants ────────────────────────────────────
#define NEX_GREEN  1339
#define NEX_YELLOW 65504
#define NEX_RED    63488
#define NEX_WHITE  65535
#define NEX_GREY   33808
#define NEX_CYAN   1055
#define NEX_DARK   10

// ── Direction mode string ───────────────────────────────────────
String getDirectionText(int dir) {
  switch (dir) {
    case 1: return "REVERSE";
    case 2: return "NEUTRAL";
    case 3: return "FORWARD";
    default: return "--";
  }
}

int getDirectionColour(int dir) {
  switch (dir) {
    case 1: return NEX_RED;
    case 2: return NEX_YELLOW;
    case 3: return NEX_GREEN;
    default: return NEX_GREY;
  }
}

// ── Challenge mode string ───────────────────────────────────────
String getChallengeText(int mode) {
  switch (mode) {
    case 1: return "Throttle Ctrl";
    case 2: return "Speed Ctrl";
    case 3: return "Autostop";
    case 4: return "Regen";
    case 5: return "Traction";
    default: return "--";
  }
}

// ── Condition mode string ───────────────────────────────────────
String getConditionText(int mode) {
  switch (mode) {
    case 1: return "Dry";
    case 2: return "Light Wet";
    case 3: return "Wet";
    case 4: return "Very Wet";
    case 5: return "Extreme";
    default: return "--";
  }
}

// ── Operation mode string ───────────────────────────────────────
String getOpModeText(uint8_t mode) {
  switch (mode) {
    case MODE_STOPPED:     return "Stopped";
    case MODE_PREOP:       return "Pre-Op";
    case MODE_OPERATIONAL: return "Operational";
    default: return "--";
  }
}

int getOpModeColour(uint8_t mode) {
  switch (mode) {
    case MODE_STOPPED:     return NEX_RED;
    case MODE_PREOP:       return NEX_YELLOW;
    case MODE_OPERATIONAL: return NEX_CYAN;
    default: return NEX_GREY;
  }
}

// ── Autostop pill — activates when challenge mode = 3 ──────────
void updateAutostopPill(int challengeMode) {
  if (challengeMode == 3) {
    sendColour("t_autostop", "bco", NEX_GREEN);
    sendColour("t_autostop", "pco", 0);
  } else {
    sendColour("t_autostop", "bco", NEX_DARK);
    sendColour("t_autostop", "pco", NEX_GREY);
  }
  refreshComponent("t_autostop");
}

// ── Main Nextion update function ────────────────────────────────
void updateNextion() {

  // ── SPEED (from pot for now — replace with od_true_speed later)
  int raw = analogRead(34);
  int speed = map(raw, 0, 4095, 0, 15);
  if (speed != prevSpeed) {
    sendText("t_speed", String(speed) + " km/h");
    sendProgressBar("j_speed", map(speed, 0, 15, 0, 100));
    prevSpeed = speed;
  }

  // ── THROTTLE
  int throttlePct = map(od_motor_command, 0, 1023, 0, 100);
  if (throttlePct != prevThrottle) {
    sendText("t_throttle", String(throttlePct) + " %");
    sendProgressBar("j_throttle", throttlePct);
    prevThrottle = throttlePct;
  }

  // ── BRAKE
  int brakePct = map(od_regen_brake, 0, 1023, 0, 100);
  if (brakePct != prevBrake) {
    sendText("t_brakepct", String(brakePct) + " %");
    sendProgressBar("j_brake", brakePct);
    prevBrake = brakePct;
  }

  // ── BRAKE STATUS
  if (od_service_brake_dc != prevBrakeStatus) {
    bool applied = (od_service_brake_dc == 0);
    sendText("t_brakestatus", applied ? "Applied" : "Released");
    sendColour("t_brakestatus", "pco", applied ? NEX_RED : NEX_GREEN);
    refreshComponent("t_brakestatus");
    prevBrakeStatus = od_service_brake_dc;
  }

  // ── DIRECTION MODE
  if (od_direction_mode != prevDirection) {
    String dirText = getDirectionText(od_direction_mode);
    int dirColour = getDirectionColour(od_direction_mode);
    sendText("t_direction", dirText);
    sendColour("t_direction", "pco", dirColour);
    refreshComponent("t_direction");
    prevDirection = od_direction_mode;
  }

  // ── CHALLENGE MODE
  if (od_challenge_mode != prevChallenge) {
    sendText("t_challenge", getChallengeText(od_challenge_mode));
    updateAutostopPill(od_challenge_mode);
    prevChallenge = od_challenge_mode;
  }

  // ── CONDITION MODE
  if (od_condition_mode != prevCondition) {
    sendText("t_condition", getConditionText(od_condition_mode));
    prevCondition = od_condition_mode;
  }

  // ── OPERATION MODE
  if (nodeOperatingMode != prevOpMode) {
    sendText("t_opmode", getOpModeText(nodeOperatingMode));
    sendColour("t_opmode", "pco", getOpModeColour(nodeOperatingMode));
    refreshComponent("t_opmode");
    prevOpMode = nodeOperatingMode;
  }
}

// ── Setup ───────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  nextionSerial.begin(115200, SERIAL_8N1, 18, 17);
}

// ── Loop ────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;
    updateNextion();
  }
}

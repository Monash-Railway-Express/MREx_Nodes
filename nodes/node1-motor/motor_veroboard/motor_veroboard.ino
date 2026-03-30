/**
 * @file motor_veroboard.ino
 * @brief Motor Veroboard file
 *
 * @details
 * This file is for the motor controller node which connects the motor controllers
 * and reversing contactors to the wider CAN Bus network.
 *
 * @author  Chiara Gillam
 *
 * @date 1/03/2026
 *
 * @version 1.0.2
 * 
 * @organisation MREX
 * 
 * @see motor_veroboard.h
 * @see CHANGELOG.md
 */


#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include <Arduino.h>
#include "motor_veroboard.h"

// User code begin: ------------------------------------------------------

// --- CAN MREx initialisation ---
uint8_t nodeID = 1;  // Change this to set your device's node ID

// --- OD definitions ---
uint16_t od_motor_command = 0;
uint16_t od_regen_brake = 0;
uint8_t od_service_brake = 0;
uint8_t od_direction_mode = 1;

// --- Setting PWM properties ---
const int freq = 5000;
const int resolution = 8;

// User code end ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);

  //Pin CANMREX to core
  xTaskCreatePinnedToCore(
    CAN_Task,
    "CAN Task",
    4096,
    &nodeID,   // <--- passed into pvParameters
    3,
    NULL,
    0
  );

  // User code Setup Begin: -------------------------------------------------
  
  // --- Register OD entries ---
  registerODEntry(0x60FF, 0x00, 2, sizeof(od_motor_command), &od_motor_command);
  registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);
  registerODEntry(0x3012, 0x01, 2, sizeof(od_service_brake), &od_service_brake);
  registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode), &od_direction_mode);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);  // COB-ID, transType, inhibit, event
  
  PdoMapEntry tpdoEntries[] = {
      {0x3012, 0x01, 8},  // Sending service brake
    };
  mapTPDO(0, tpdoEntries, 1); //TPDO 1, entries, num entries

  // --- Register RPDOs ---
  configureRPDO(0, 0x180 + 3, 255, 0);         // COB-ID, transType, inhibit

  PdoMapEntry rpdoEntries[] = {
    {0x60FF, 0x00, 16},  // Example: index 0x2000, subindex 1, 16 bits
    {0x3012, 0x00, 16}    // Example: index 0x2001, subindex 0, 16 bits
  };
  mapRPDO(0, rpdoEntries, 2);

  // --- Set pin modes ---
  ledcAttach(MOTOR_PIN, freq, resolution);
  ledcAttach(REGEN_BRAKE_PIN, freq, resolution);
  pinMode(REVERSING_PIN, OUTPUT);

  // User code Setup end ------------------------------------------------------
}


void loop() {
  OperatingMode mode = static_cast<OperatingMode>(nodeOperatingMode); // Uses nodeOperatingMode to decode what operating mode we're in

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
      StoppedMode(); // fail-safe
      break;
  }
}


void StoppedMode(){
  ledcWrite(MOTOR_PIN, 0);
  ledcWrite(REGEN_BRAKE_PIN, 0);
}


void PreOpMode(){
  ledcWrite(MOTOR_PIN, 0);
  ledcWrite(REGEN_BRAKE_PIN, 0);
  
  // Set direction mode 
  if (od_direction_mode == 1) {
    digitalWrite(REVERSING_PIN, LOW);
  } else if (od_direction_mode == 3) {
    digitalWrite(REVERSING_PIN, HIGH);
  }

  DebugPreOpOutput();
}


void OperationalMode() {
  
  //Timing for 10Hz updates to motor and regen brakes
  static unsigned long millisPrev = 0;
  static const long interval = 100;
  unsigned long currentMillis = millis();

  // Lockout for preventing instant throttle up when regen brakes released
  static uint8_t motor_lockout = 1; // 1 motor locked out / 0 motor unlocked

  if (currentMillis - millisPrev >= interval) {
    millisPrev = currentMillis;

    // Lockout logic
    if (od_motor_command > 10 && od_regen_brake > 10) {
      motor_lockout = 1;
    } else if (od_motor_command <= 10 && od_regen_brake <= 10) {
      motor_lockout = 0;
    }


    uint8_t motorpwmValue = 0;
    uint8_t brakepwmValue = 0;
    if (od_motor_command > 10 && od_regen_brake <= 10 && motor_lockout == 0) {
      motorpwmValue = od_motor_command >> 2;
      brakepwmValue = 0;
    }
    else if (od_motor_command > 10 && od_regen_brake > 10) {
      motorpwmValue = 0;
      brakepwmValue = od_regen_brake >> 2;
    }
    else if (od_motor_command <= 10 && od_regen_brake > 10) {
      motorpwmValue = 0;
      brakepwmValue = od_regen_brake >> 2;
    }
    else {
      motorpwmValue = 0;
      brakepwmValue = 0;
    }

    ledcWrite(MOTOR_PIN, motorpwmValue);
    ledcWrite(REGEN_BRAKE_PIN, brakepwmValue);

    DebugOperationalOutput(motorpwmValue, brakepwmValue, od_service_brake);

  }
}


void DebugOperationalOutput(uint8_t motorpwmValue, uint8_t brakepwmValue, uint8_t od_service_brake) {
  Serial.print("Motor value: ");
  Serial.println(motorpwmValue);
  Serial.print("Brake value: ");
  Serial.println(brakepwmValue);
  Serial.print("Service brake: ");
  Serial.println(od_service_brake);
}


void DebugPreOpOutput() {
  Serial.print("Direction mode: ");
  Serial.println(od_direction_mode);
}

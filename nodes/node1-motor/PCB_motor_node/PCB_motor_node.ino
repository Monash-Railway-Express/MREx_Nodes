/**
 * PCB motor node  file 
 *
 * File:            PCB_motor_node.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    13/03/2026
 * Last Modified:   13/03/2026
 * Version:         1.0.1
 *
 */

#include <CAN_MREx.h>
#include <Arduino.h>
#include <Wire.h>
#include "driver/pcnt.h"

// --- CAN MREx initialisation ---
const uint8_t nodeID = 1;

// --- CAN Definitions ---
#define TX_GPIO_NUM GPIO_NUM_48   // CAN TX
#define RX_GPIO_NUM GPIO_NUM_47   // CAN RX

// DAC I2C pins 
#define SDA_PIN GPIO_NUM_2
#define SCL_PIN GPIO_NUM_1

//PINS
#define THROTTLE_SWITCH GPIO_NUM_4
#define REVERSING_SWITCH_MC GPIO_NUM_5
#define BRAKE_SWITCH GPIO_NUM_6
#define MOTOR1_GREEN_LED GPIO_NUM_7
#define MOTOR2_GREEN_LED GPIO_NUM_15
#define MOTOR1_RED_LED GPIO_NUM_16
#define MOTOR2_RED_LED GPIO_NUM_17
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_8
#define ENCODER_Z GPIO_NUM_9
#define REVERSING_CONTACTOR GPIO_NUM_10

//EXTRA GPIO include GPIO11, GPIO12, GPIO48, GPIO47, GPIO21

//Setup pulse counter for the Encoder
#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_CHANNEL PCNT_CHANNEL_0
#define PCNT_HIGH_LIMIT 1000000
#define PCNT_LOW_LIMIT 0
const float pulsesPerRev = 200.0f;
float wheelCircumference_m = 0.3; 

//Functions
void commonConfig();
void writeDAC(uint8_t channel, uint16_t value);
void setupPCNT();

// --- PI Controller Variables ---
float Kp = 20.0f;        // proportional gain (tune)
float Ki = 5.0f;         // integral gain (tune)
float integrator = 0.0f; // integral accumulator
float integratorLimit = 300.0f; // anti-windup clamp

// --- Brake Threshold ---
const float serviceBrakeSpeedThreshold = 2.0f;   // km/h, adjust as needed


// --- OD definitions ---
uint16_t desiredSpeed = 0;
uint16_t regenBrake = 0;
uint8_t serviceBrake = 0;

// PID Timing
unsigned long previousMillis = 0;
const long interval = 100; // 100 ms

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");

  // Initialise CAN MREx
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);
  
  //Initialise I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  //Setup The DAC
  commonConfig();   // power up VOUT0/1, enable internal ref

  //Setup the pulse counter
  setupPCNT();

  // --- Register OD entries ---
  registerODEntry(0x60FF, 0x00, 2, sizeof(desiredSpeed), &desiredSpeed);
  registerODEntry(0x3012, 0x00, 2, sizeof(regenBrake), &regenBrake);
  registerODEntry(0x3012, 0x01, 2, sizeof(serviceBrake), &serviceBrake);

  // --- Register TPDOs ---
  configureTPDO(0, 0x180 + nodeID, 255, 100, 100);
  PdoMapEntry tpdoEntries[] = {
      {0x3012, 0x01, 8},
  };
  mapTPDO(0, tpdoEntries, 1);

  // --- Register RPDOs ---
  configureRPDO(0, 0x180 + 3, 255, 0);
  PdoMapEntry rpdoEntries[] = {
    {0x60FF, 0x00, 16},
    {0x3012, 0x00, 16}
  };
  mapRPDO(0, rpdoEntries, 2);
}

// -------------------- LOOP --------------------
void loop() {

  // Stopped mode
  if (nodeOperatingMode == 0x02){
    handleCAN(nodeID);
  }

  // Pre-operational
  if (nodeOperatingMode == 0x80){
    handleCAN(nodeID);
  }

  // Operational
  if (nodeOperatingMode == 0x01){
    handleCAN(nodeID);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      int16_t count = 0;
      pcnt_get_counter_value(PCNT_UNIT_0, &count);
      pcnt_counter_clear(PCNT_UNIT_0);

      // Convert to speed
      float rev = count / pulsesPerRev;
      float meters = rev * wheelCircumference_m;
      float speed_mps = meters / 0.1f;   // 0.1s interval
      float speed_kmh = speed_mps * 3.6f;

      uint16_t motorDAC = 0;
      uint16_t brakeDAC = 0;

      // -------------------- SPEED CONTROL + BRAKE LOGIC --------------------

      // Compute speed error
      float error = desiredSpeed - speed_kmh;

      // Update integrator (anti-windup)
      integrator += error * Ki * 0.1f;   // 0.1s loop time
      if (integrator > integratorLimit) integrator = integratorLimit;
      if (integrator < 0) integrator = 0;

      // PI output
      float control = (error * Kp) + integrator;

      // Clamp PI output to DAC range
      if (control < 0) control = 0;
      if (control > 1023) control = 1023;

      // Default outputs (no braking)
      motorDAC = (uint16_t)control;
      brakeDAC = 0;
      serviceBrake = 0;

      // -------------------- BRAKE OVERRIDE --------------------
      if (regenBrake > 10) {

          // Disable throttle when braking
          motorDAC = 0;

          // Apply regen brake directly
          brakeDAC = regenBrake;

          // Apply service brake if speed is below threshold
          if (speed_kmh <= serviceBrakeSpeedThreshold) {
              serviceBrake = 1;
          } else {
              serviceBrake = 0;
          }
      }

      // -------------------- DAC OUTPUT --------------------
      writeDAC(0, motorDAC);   // DAC0 = motor
      writeDAC(1, brakeDAC);   // DAC1 = regen brake

      // -------------------- DEBUG --------------------
      Serial.print("Motor DAC: ");
      Serial.println(motorDAC);
      Serial.print("Brake DAC: ");
      Serial.println(brakeDAC);
      Serial.print("Service brake: ");
      Serial.println(serviceBrake);
    }
  }
}


//Setup DAC
void commonConfig() {
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x1F);   // COMMON-CONFIG register
  Wire.write(0x02);   // EN-INT-REF=0, VOUT0 ON, IOUT0 OFF
  Wire.write(0x01);   // VOUT1 ON, IOUT1 OFF
  Wire.endTransmission();
}


// -- Write function for dac
void writeDAC(uint8_t channel, uint16_t value) {
  if (value > 1023) value = 1023;
  uint16_t regVal = value << 6;                 // left-align 10-bit

  uint8_t reg = (channel == 0) ? 0x1C : 0x19;   // DAC-0-DATA / DAC-1-DATA

  Wire.beginTransmission(DAC_ADDR);
  Wire.write(reg);
  Wire.write(regVal >> 8);
  Wire.write(regVal & 0xFF);
  Wire.endTransmission();
}

// --- Setup the pulse counter
void setupPCNT() {
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = ENCODER_A;
    pcnt_config.ctrl_gpio_num = ENCODER_B;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;   // count on rising edge
    pcnt_config.neg_mode = PCNT_COUNT_DIS;   // ignore falling edge
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;  // B low = reverse
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;     // B high = forward
    pcnt_config.counter_h_lim = PCNT_HIGH_LIMIT;
    pcnt_config.counter_l_lim = PCNT_LOW_LIMIT;

    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

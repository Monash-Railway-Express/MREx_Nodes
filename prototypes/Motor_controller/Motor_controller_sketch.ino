#include <Arduino.h>
#include "driver/pcnt.h"
//debugging
#include <esp_system.h>

#define ENCODER_PIN 37
#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_CHANNEL PCNT_CHANNEL_0
#define PCNT_HIGH_LIMIT 32767
#define PCNT_LOW_LIMIT 0

//Settings
const unsigned int pulsesPerRev = 40; // Encoder pulses per revolution

// ======== PID Settings ========
float targetRPM = 1000.0; // Desired speed
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// ======== Encoder Variables ========
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long minGap = 500; // microseconds (debounce)

// ======== PID Variables ========
float integral = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;


//defining PWM settings
const int pwmFreq = 30000;  // 30 kHz for motor control
const int pwmRes = 8; // 

// ---------------------------
// Pin definitions
// ---------------------------

// Motor controllers share same control inputs (Forward, Reverse, BrakeSW, MicroSW)
// Motor controllers have separate outputs (alarms, LEDs, PWM)

// ----- Motor Controller 1 (M1) -----
const uint8_t M1_AlarmPin    = 40;   // Alarm output
const uint8_t M1_GreenLEDpin = 39;   // Normal operation indicator
const uint8_t M1_RedLEDpin   = 38;   // Fault indicator

// ----- Motor Controller 2 (M2) -----
const uint8_t M2_AlarmPin    = 36;
const uint8_t M2_GreenLEDpin = 35;
const uint8_t M2_RedLEDpin   = 0;

// ----- Shared control lines -----
const uint8_t SHARED_ForwardPin = 5;   
const uint8_t SHARED_ReversePin = 15;  
const uint8_t SHARED_BrakeSW    = 16;  
const uint8_t SHARED_MicroSW    = 17;  
const uint8_t THROTTLE_PIN = 18;  // PWM output -> Throttle analog input
const uint8_t BRAKE_PIN    = 9;  // PWM output -> Brake analog input

// ---------------------------
// Status helpers
// ---------------------------

bool M1AlarmState = LOW;
bool M2AlarmState = LOW;
bool M1GreenState = LOW;
bool M1RedState   = LOW;
bool M2GreenState = LOW;
bool M2RedState   = LOW;

//debugging code

void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch (reason) {
    case ESP_RST_UNKNOWN: Serial.println("UNKNOWN"); break;
    case ESP_RST_POWERON: Serial.println("POWERON"); break;
    case ESP_RST_EXT: Serial.println("EXTERNAL_RESET"); break;
    case ESP_RST_SW: Serial.println("SOFTWARE_RESET"); break;
    case ESP_RST_PANIC: Serial.println("PANIC (exception)"); break;
    case ESP_RST_INT_WDT: Serial.println("INTERRUPT WDT"); break;
    case ESP_RST_TASK_WDT: Serial.println("TASK WDT"); break;
    case ESP_RST_WDT: Serial.println("WDT"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("DEEPSLEEP"); break;
    case ESP_RST_BROWNOUT: Serial.println("BROWNOUT"); break;
    case ESP_RST_SDIO: Serial.println("SDIO"); break;
    default:
      Serial.printf("code=%d\n", reason);
  }
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Traction MCU initializing (ESP32-S3, arduino-esp32 v3.x LEDC API)...");

  printResetReason();
  delay(200);
  // Status inputs
  pinMode(M1_AlarmPin, INPUT);
  pinMode(M1_GreenLEDpin, INPUT);
  pinMode(M1_RedLEDpin, INPUT);
  pinMode(M2_AlarmPin, INPUT);
  pinMode(M2_GreenLEDpin, INPUT);
  pinMode(M2_RedLEDpin, INPUT);

  // Single encoder
  pinMode(ENCODER_PIN, INPUT);

  //setting up PWM for controling throttle amount
  ledcAttach(THROTTLE_PIN, pwmFreq, pwmRes);
  ledcAttach(BRAKE_PIN, pwmFreq, pwmRes);

  // Shared outputs
  pinMode(SHARED_ForwardPin, OUTPUT);
  pinMode(SHARED_ReversePin, OUTPUT);
  pinMode(SHARED_BrakeSW, OUTPUT);
  pinMode(SHARED_MicroSW, OUTPUT);

  // Init outputs to zero (use pwmWritePin directly)
  ledcWrite(THROTTLE_PIN,0);
  ledcWrite(BRAKE_PIN,0);

  Serial.println("Setup complete. PWM pins attached (using ledcAttach()).");

  //Configuring PCNT counting
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_PIN;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.channel = PCNT_CHANNEL;
  pcnt_config.unit = PCNT_UNIT;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = 32767;
  pcnt_config.counter_l_lim = 0;

  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

}

// ---------------------------
// Main loop
// ---------------------------

unsigned long lastPrint = 0;
void loop() {

  // Reads status signals of controllers, if active, motor controller has thrown error code

  // M1GreenState = digitalRead(M1_GreenLEDpin);
  // M1RedState   = digitalRead(M1_RedLEDpin);
  // M2GreenState = digitalRead(M2_GreenLEDpin);
  // M2RedState   = digitalRead(M2_RedLEDpin);

  // Writing to Controller pin

  // digitalWrite(SHARED_ForwardPin, LOW);
  // digitalWrite(SHARED_ReversePin, LOW);
  // digitalWrite(SHARED_BrakeSW, LOW);
  // digitalWrite(SHARED_MicroSW, LOW);

  //Grabs and stores PCNT count value, can be converted to distance using wheel circumference

  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  Serial.printf("Count: %d\n", count);
  delay(1000);

  //Use this to write PWM signal to Throttle / Brake analog input inputs 

  // 128 gives duty cycle of ~50%
  ledcWrite(THROTTLE_PIN,128);
  ledcWrite(BRAKE_PIN,128);

}

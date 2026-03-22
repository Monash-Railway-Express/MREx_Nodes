// ESP32-S3 Arduino sketch: PCNT counting (rising edge only) + PID speed control
#include <Arduino.h>
#include "driver/pcnt.h"
#include <esp_system.h>

// ---------------------------
// Hardware configuration
// ---------------------------
#define ENCODER_PIN 37
#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_CHANNEL PCNT_CHANNEL_0
#define PCNT_HIGH_LIMIT 1000000
#define PCNT_LOW_LIMIT 0

const unsigned int pulsesPerRev = 40; // Encoder pulses per revolution

// LEDC (PWM) configuration
const int pwmFreq = 30000;  // 30 kHz
const int pwmRes = 8;       // 8-bit resolution => duty: 0..255


// Pin defs (kept from your original)
const uint8_t M1_AlarmPin    = 40;
const uint8_t M1_GreenLEDpin = 39;
const uint8_t M1_RedLEDpin   = 38;
const uint8_t M2_AlarmPin    = 36;
const uint8_t M2_GreenLEDpin = 35;
const uint8_t M2_RedLEDpin   = 0;

const uint8_t SHARED_ForwardPin = 5;
const uint8_t SHARED_ReversePin = 15;
const uint8_t SHARED_BrakeSW    = 16;
const uint8_t SHARED_MicroSW    = 17;
const uint8_t THROTTLE_PIN = 18;  // pin -> attach to LEDC channel
const uint8_t BRAKE_PIN    = 9;

// ---------------------------
// PID settings
// ---------------------------
float targetRPM = 1000.0; // Desired speed
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

float integral = 0.0;
float lastError = 0.0;
unsigned long lastPIDMicros = 0;
const unsigned long pidIntervalMs = 100; // PID update interval (ms)

// Output limits (for 8-bit PWM)
const float outMin = 0.0;
const float outMax = 255.0;

// ---------------------------
// Utility: print reset reason
// ---------------------------
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
// PCNT (pulse counter) setup
// ---------------------------
void setupPCNT() {
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_PIN;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.channel = PCNT_CHANNEL;
  pcnt_config.unit = PCNT_UNIT;
  pcnt_config.pos_mode = PCNT_COUNT_INC;   // count on rising edge
  pcnt_config.neg_mode = PCNT_COUNT_DIS;   // ignore falling edge
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = PCNT_HIGH_LIMIT;
  pcnt_config.counter_l_lim = PCNT_LOW_LIMIT;

  esp_err_t err = pcnt_unit_config(&pcnt_config);
  if (err != ESP_OK) {
    Serial.printf("pcnt_unit_config failed: %d\n", err);
  }

  // Disable PCNT filter so we don't accidentally drop legitimate pulses.
  // (If your SDK doesn't provide pcnt_filter_disable, you can set a very small filter value.)
  pcnt_filter_disable(PCNT_UNIT);

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

// Read & clear PCNT counter; returns pulses since last read
long readAndClearPCNT() {
  int16_t cnt = 0;
  if (pcnt_get_counter_value(PCNT_UNIT, &cnt) != ESP_OK) {
    return 0;
  }
  pcnt_counter_clear(PCNT_UNIT);
  return (long)cnt;
}

// Convert pulses counted in intervalMs to RPM
float pulsesToRPM(long deltaPulses, float intervalMs) {
  if (intervalMs <= 0.0f) return 0.0f;
  float pulsesPerMinute = (float)deltaPulses * (60000.0f / intervalMs);
  return pulsesPerMinute / (float)pulsesPerRev;
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Traction MCU initializing (ESP32-S3, Arduino LEDC API)...");

  printResetReason();
  delay(200);

  // Status inputs
  pinMode(M1_AlarmPin, INPUT);
  pinMode(M1_GreenLEDpin, INPUT);
  pinMode(M1_RedLEDpin, INPUT);
  pinMode(M2_AlarmPin, INPUT);
  pinMode(M2_GreenLEDpin, INPUT);
  pinMode(M2_RedLEDpin, INPUT);

  // Encoder pin (input only; no dynamic debounce)
  pinMode(ENCODER_PIN, INPUT);

  // Setup LEDC (Arduino ESP32 API)
  ledcAttach(THROTTLE_PIN, pwmFreq, pwmRes);
  ledcAttach(BRAKE_PIN, pwmFreq, pwmRes);

  // Shared outputs
  pinMode(SHARED_ForwardPin, OUTPUT);
  pinMode(SHARED_ReversePin, OUTPUT);
  pinMode(SHARED_BrakeSW, OUTPUT);
  pinMode(SHARED_MicroSW, OUTPUT);

  // Init outputs to zero
  ledcWrite(THROTTLE_PIN, 0);
  ledcWrite(BRAKE_PIN, 0);

  Serial.println("Setup complete. PWM channels attached.");

  // Setup PCNT
  setupPCNT();

  // Initialize PID timing
  lastPIDMicros = micros();
}

// ---------------------------
// Main loop
// ---------------------------
void loop() {
  unsigned long nowMicros = micros();

  // Non-blocking: update PID every pidIntervalMs, 
  // can reduce this interval if wanted, could even have PID loop update on each encoder pulse.
  if ((nowMicros - lastPIDMicros) >= (pidIntervalMs * 1000UL)) {
    unsigned long elapsedMicros = nowMicros - lastPIDMicros;
    float elapsedMs = elapsedMicros / 1000.0f;

    // Read PCNT and clear for next interval,
    // this also prevent the Encoder count register from overflowing
    long deltaPulses = readAndClearPCNT();

    // Convert to RPM, basic RPM calculation
    float measuredRPM = pulsesToRPM(deltaPulses, elapsedMs);

    // error calc
    float error = targetRPM - measuredRPM;

    // Integrate, just adds area of error vs time each interval
    integral += error * (elapsedMs / 1000.0f);
    // clamp integral to avoid runaway,
    // sometime integral can go into a positive feedback loop
    float integralMax = 1000.0;
    if (integral > integralMax) integral = integralMax;
    if (integral < -integralMax) integral = -integralMax;

    // derivative (rpm per second)
    float derivative = 0.0f;
    if (elapsedMs > 0.0f) {
      derivative = (error - lastError) / (elapsedMs / 1000.0f);
    }

    // PID output
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Clamp output to PWM range
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    // Apply to throttle channel (8-bit)
    uint32_t duty = (uint32_t)constrain(round(output), (int)outMin, (int)outMax);
    ledcWrite(THROTTLE_PIN, 127);

    // Save state for next iteration
    lastError = error;
    lastPIDMicros = nowMicros;

    // Debug print
    Serial.printf("dt=%.1fms pulses=%ld RPM=%.1f tgt=%.1f out=%.1f duty=%u\n",
                  elapsedMs, deltaPulses, measuredRPM, targetRPM, output, duty);
  }

  // Keep loop responsive
  delay(2);
}

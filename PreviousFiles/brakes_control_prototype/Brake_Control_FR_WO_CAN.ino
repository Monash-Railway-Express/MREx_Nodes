#include <Arduino.h>
#include "esp_task_wdt.h"  // Include ESP32 watchdog library

// Pin assignments
const int RELAY_PIN = 22;
const int LED_PIN = 32; // LED indicator
const int BRAKE1_SENSOR_PIN = 26; // Current sensor for Brake 1
const int BRAKE2_SENSOR_PIN = 27; // Current sensor for Brake 2
const int CAN_TX_PIN = 18; // CAN TX
const int CAN_RX_PIN = 19; // CAN RX

// Shared variables
bool brake1On = false;
bool brake2On = false;
bool relayState = false; // false = brakes ON (coil OFF), true = brakes OFF (coil ON)
bool brakeFault = false;
bool sensorFault = false;
unsigned long lastCommandTime = 0;

SemaphoreHandle_t xMutex; // Mutex for shared resources

// Settings
const int NUM_SAMPLES = 10; // For averaging
const int ON_THRESHOLD = 2190;  // Below this = brake ON
const int OFF_THRESHOLD = 1960; // Above this = brake OFF
const unsigned long PRINT_INTERVAL = 1000; // 1 second
const unsigned long FAILSAFE_TIMEOUT = 60000; // 5 minute
const int SENSOR_FAULT_LIMIT = 4090; // ADC near max indicates fault
const int SENSOR_FAULT_MIN = 10;     // ADC near zero indicates fault

// Task 1: Handle Serial Commands
void TaskSerialHandler(void *pvParameters) {
  Serial.println("[TaskSerialHandler] Started");
  for (;;) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        if (command.equalsIgnoreCase("ON")) {
          relayState = false; // Brakes ON (coil OFF)
          digitalWrite(RELAY_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          Serial.println("Brakes commanded ON!");
        } else if (command.equalsIgnoreCase("OFF")) {
          relayState = true; // Brakes OFF (coil ON)
          digitalWrite(RELAY_PIN, HIGH);
          digitalWrite(LED_PIN, HIGH);
          Serial.println("Brakes commanded OFF!");
        } else {
          Serial.println("Invalid command. Type 'ON' or 'OFF'.");
        }
        lastCommandTime = millis(); // Reset failsafe timer
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Check every 50ms
  }
}

// Task 2: Monitor Current Sensors and Detect Faults
void TaskBrakeMonitor(void *pvParameters) {
  Serial.println("[TaskBrakeMonitor] Started");
  unsigned long lastPrintTime = 0;

  for (;;) {
    long sum1 = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      sum1 += analogRead(BRAKE1_SENSOR_PIN);
      delay(2);
    }
    int avgBrake1 = sum1 / NUM_SAMPLES;

    long sum2 = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      sum2 += analogRead(BRAKE2_SENSOR_PIN);
      delay(2);
    }
    int avgBrake2 = sum2 / NUM_SAMPLES;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      if ((avgBrake1 >= SENSOR_FAULT_LIMIT || avgBrake1 <= SENSOR_FAULT_MIN) ||
          (avgBrake2 >= SENSOR_FAULT_LIMIT || avgBrake2 <= SENSOR_FAULT_MIN)) {
        sensorFault = true;
      } else {
        sensorFault = false;
      }

      if (avgBrake1 < ON_THRESHOLD) {
        brake1On = true;
      } else if (avgBrake1 > OFF_THRESHOLD) {
        brake1On = false;
      }

      if (avgBrake2 < ON_THRESHOLD) {
        brake2On = true;
      } else if (avgBrake2 > OFF_THRESHOLD) {
        brake2On = false;
      }

      if (!sensorFault) {
        if (relayState == false) {
          brakeFault = (!brake1On || !brake2On);
        } else {
          brakeFault = (brake1On || brake2On);
        }
      } else {
        brakeFault = false;
      }

      xSemaphoreGive(xMutex);
    }

    if (millis() - lastPrintTime >= PRINT_INTERVAL) {
      lastPrintTime = millis();
      Serial.print("Brake 1 ADC: ");
      Serial.print(avgBrake1);
      Serial.print(" | Detected: ");
      Serial.println(brake1On ? "ON" : "OFF");

      Serial.print("Brake 2 ADC: ");
      Serial.print(avgBrake2);
      Serial.print(" | Detected: ");
      Serial.println(brake2On ? "ON" : "OFF");

      Serial.print("Commanded State: ");
      Serial.print(relayState ? "OFF (coil ON)" : "ON (coil OFF)");
      Serial.print(" | Brake Fault: ");
      Serial.print(brakeFault ? "YES" : "NO");
      Serial.print(" | Sensor Fault: ");
      Serial.println(sensorFault ? "YES" : "NO");
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// Task 3: CAN Bus Placeholder
void TaskCANHandler(void *pvParameters) {
  Serial.println("[TaskCANHandler] Started (Placeholder for future CAN logic)");
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task 4: Failsafe Logic
void TaskFailsafe(void *pvParameters) {
  Serial.println("[TaskFailsafe] Started");
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      if ((millis() - lastCommandTime) > FAILSAFE_TIMEOUT) {
        if (relayState) {
          relayState = false;
          digitalWrite(RELAY_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          Serial.println("⚠️ Failsafe triggered: No command for 1 minute. Brakes ENGAGED!");
        }
      }
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Type 'ON' or 'OFF' to control brakes.");

  lastCommandTime = millis();
  xMutex = xSemaphoreCreateMutex();

  // ✅ Initialize Watchdog with new API
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 5000,      // 5 seconds
      .idle_core_mask = (1 << 0) | (1 << 1), // Monitor both cores
      .trigger_panic = true    // Reset CPU on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // Add main loop task to watchdog

  xTaskCreatePinnedToCore(TaskSerialHandler, "SerialHandler", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskBrakeMonitor, "BrakeMonitor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANHandler, "CANHandler", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskFailsafe, "Failsafe", 2048, NULL, 1, NULL, 0);
}


void loop() {
  // ✅ Feed watchdog regularly
  esp_task_wdt_reset();
  delay(100);
}
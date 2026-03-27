#include "driver/twai.h"  // Native ESP-IDF CAN driver

#define CAN_TX 17
#define CAN_RX 16

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting CAN...");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); // Enable the CAN transceiver (if required)


  // ✅ Cast the GPIO numbers properly
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();               // 500 kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();             // Accept all IDs

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("CAN driver started");
  } else {
    Serial.println("CAN init failed");
    while (1);
  }
}

void loop() {
  twai_message_t msg;
  // msg.identifier = 0x123;
  // msg.data_length_code = 1;
  // msg.data[0] = 0xAB;
  // msg.flags = 0;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(1000));
  if (result == ESP_OK) {
    Serial.println("Sent 0xAB");
  } else {
    Serial.println("Failed to send");
  }

  delay(1000);
}

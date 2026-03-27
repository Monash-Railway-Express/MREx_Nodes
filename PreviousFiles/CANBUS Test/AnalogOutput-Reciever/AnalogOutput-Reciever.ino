#include "driver/twai.h"

#define CAN_TX 5
#define CAN_RX 4
#define DAC_OUT_PIN 25

void setup() {
  Serial.begin(9600);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("CAN receiver ready");
  } else {
    Serial.println("CAN init failed");
    while (1);
  }
}

void loop() {
  twai_message_t msg;

  if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (msg.identifier == 0x101 && msg.data_length_code == 1) {
      uint8_t analogByte = msg.data[0];  // Get 0–255 value
      dacWrite(DAC_OUT_PIN, analogByte);  // Output to DAC

      // Debug print
      float voltageOut = analogByte * (3.3 / 255.0);
      Serial.printf("Received: %d → DAC = %.2fV\n", analogByte, voltageOut);
    }
  }
}
